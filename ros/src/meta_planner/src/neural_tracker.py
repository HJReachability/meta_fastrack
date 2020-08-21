# Copyright (c) 2019, The Regents of the University of California (Regents).
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#    1. Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Please contact the author(s) of this library if you have any questions.
# Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
#
################################################################################

import rospy
import tensorflow as tf
import neural_utils
import pickle
import numpy as np
import itertools

from neural_policy import NeuralPolicy
from neural_value_function import NeuralValueFunction
from fastrack_srvs.srv import KinematicPlannerDynamics
from fastrack_srvs.srv import TrackingBoundBox
from fastrack_msgs.msg import Control
from fastrack_msgs.msg import State
from meta_planner_msgs.msg import PlannerState
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker

class NeuralTracker(object):
    def __init__(self):
        self._ready = False
        self._initialized = False
        self._planner_x = None
        self._tracker_x = None

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/neural_tracker"

        if not self.LoadParameters():
            return False

        if not self.RegisterCallbacks():
            return False

        self._values = []
        for ii in range(self._num_planners**2):
            self._values.append(NeuralValueFunction())
            if not self._values[-1].InitializeFromFile(self._network_files[ii]):
                return False


        self._initialized = True
        return True

    # Load parameters.
    def LoadParameters(self):
        if not rospy.has_param("~topic/ready"):
            return False
        self._ready_topic = rospy.get_param("~topic/ready")

        if not rospy.has_param("~topic/tracker_state"):
            return False
        self._tracker_state_topic = rospy.get_param("~topic/tracker_state")

        if not rospy.has_param("~topic/planner_state"):
            return False
        self._planner_state_topic = rospy.get_param("~topic/planner_state")

        if not rospy.has_param("~topic/control"):
            return False
        self._control_topic = rospy.get_param("~topic/control")

        if not rospy.has_param("~vis/bound"):
            return False
        self._bound_topic = rospy.get_param("~vis/bound")

        if not rospy.has_param("~srv/bound"):
            return False
        self._bound_names = rospy.get_param("~srv/bound")

        if not rospy.has_param("~srv/planner_dynamics"):
            return False
        self._planner_dynamics_names = rospy.get_param("~srv/planner_dynamics")

        if not rospy.has_param("~frames/planner"):
            return False
        self._planner_frame = rospy.get_param("~frames/planner")

        if not rospy.has_param("~time_step"):
            return False
        self._time_step = rospy.get_param("~time_step")

        if not rospy.has_param("~num_planners"):
            return False
        self._num_planners = rospy.get_param("~num_planners")

        if not rospy.has_param("~values/network_files"):
            return False
        self._network_files = rospy.get_param("~values/network_files")
        assert(len(self._network_files) == self._num_planners)

        return True

    # Register callbacks.
    def RegisterCallbacks(self):
        # Subscribers.
        self._ready_sub = rospy.Subscriber(
            self._ready_topic, Empty, self.ReadyCallback())
        self._planner_state_sub = rospy.Subscriber(
            self._planner_state_topic, PlannerState, self.PlannerStateCallback())
        self._tracker_state_sub = rospy.Subscriber(
            self._tracker_state_topic, State, self.TrackerStateCallback())

        # Publishers.
        self._control_pub = rospy.Publisher(self._control_topic, Control)
        self._bound_pub = rospy.Publisher(self._bound_topic, Marker)

        # Timer.
        self._timer = rospy.Timer(
            rospy.Duration(self._time_step), self.TimerCallback())

        # Services as lambdas.
        self._bound_srvs = []
        self._planner_dynamics_srvs = []
        for ii in range(len(self._bound_names)):
            bound_srv_name = self._bound_names[ii]
            bound_callback = lambda req:
                return self._values[ii].TrackingBound()

            self._bound_srvs.append(rospy.Service(
                bound_srv_name, TrackingBoundBox, bound_callback))

            dynamics_srv_name = self._planner_dynamics_names[ii]
            dynamics_callback = lambda req:
                return self._values[ii].PlannerDynamics()

            self._planner_dynamics_srvs.append(rospy.Service(
                dynamics_srv_name, KinematicPlannerDynamics, dynamics_callback))

        return True

    # Is the system ready?
    def ReadyCallback(self, msg):
        self._ready = True

    # Update tracker/planner states.
    def TrackerStateCallback(self, msg):
        self._tracker_x = msg
    def PlannerStateCallback(self, msg):
        self._planner_x = msg
        self._flattened_value_id = self.FromRowMajor(
            msg.previous_planner_id, msg.next_planner_id)

    # Convert previous/next planner ids to flattened id for value fn matrix.
    # NOTE: row = previous, column = next.
    def FromRowMajor(self, row, col):
        return self._num_planners * row + col

    # Timer callback - compute optimal control and publish.
    def TimerCallback(self, event):
        if not self._ready:
            return

        if self._tracker_x is None or self._planner_x is None:
            rospy.logwarn_throttle(
                1.0, "%s: Have not received planner/tracker state yet." % self._name)
            return

        # Publish control.
        # NOTE: sending previous planner state because previous and next are
        # identical when interpolated.
        self._control_pub.publish(
            self._values[self._flattened_value_id].OptimalControl(
                self._tracker_x, self._planner_x.previous_planner_state))

        # Publish bound.
        bound = self._values[self._flattened_value_id].TrackingBound()
        marker = Marker()
        marker.header.frame_id = self._planner_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "tracking bound";
        marker.id = 0;
        marker.type = visualization_msgs.Marker.CUBE;
        marker.action = visualization_msgs.Marker.ADD;
        marker.color.a = 0.3;
        marker.color.r = 0.5;
        marker.color.g = 0.1;
        marker.color.b = 0.5
        marker.scale.x = 2.0 * bound.x;
        marker.scale.y = 2.0 * bound.y;
        marker.scale.z = 2.0 * bound.z;

        self._bound_pub.publish(marker)
