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
from fastrack_srvs.srv import KinematicPlannerDynamicsResponse
from fastrack_srvs.srv import TrackingBoundBoxResponse
from fastrack_msgs.msg import Control

# Network files (param) format:
# List of picklefiles

# Picklefile format:
# {"weights":<np.array>,
#  "layers":<[list ints]>,
#  "control_bounds_upper":<[list of doubles]>
#  "control_bounds_lower":<[list of doubles]>
#  "tracking_error_bound":<[list of doubles]>
#  "planner_params": {
#                     "max_speed":[list doubles],
#                     "max_vel_dist":[list doubles],
#                     "max_acc_dist":[list doubles]
# }


class NeuralValueFunction(object):
    # Static variable for indexing how many of these are floating around out there.
    _registry_index = 0

    def __init__(self):
        self._registry_index = NeuralValueFunction._registry_index
        self._initialized = False
        NeuralValueFunction._registry_index += 1

    # Initialization and loading parameters.
    def InitializeFromFile(self, network_file):
        self._name = rospy.get_name() + "/neural_value"

        sess = tf.Session();
        self.policy = NeuralPolicy(
            network_file, self._registry_index, sess=sess, ppick=15, pick_=15)

        self._initialized = True
        return True

    def OptimalControl(self, tracker_x_msg, planner_x_msg):
        tracker_x = neural_utils.UnpackState(tracker_x_msg)
        planner_x = neural_utils.UnpackState(planner_x_msg)

        # Some disgusting relative state conversions.
        # TODO! Please God come up with a better way to do this.
        if (self.policy.TrackerStateType() == "PositionVelocityYaw" and
            self.policy.PlannerStateType() == "PositionVelocity" and
            self.policy.IsPlannerKinematic()):
            relative_x = np.array([
                tracker_x[0] - planner_x[0],
                tracker_x[1] - planner_x[1],
                tracker_x[2] - planner_x[2],
                tracker_x[3],
                tracker_x[4],
                tracker_x[5],
                tracker_x[6]
            ])
        else:
            assert(self.policy.TrackerStateType() == "PositionVelocityYaw" and
                   self.policy.PlannerStateType() == "PositionVelocity" and
                   not self.policy.IsPlannerKinematic())
            relative_x = np.array([
                tracker_x[0] - planner_x[0],
                tracker_x[1] - planner_x[1],
                tracker_x[2] - planner_x[2],
                tracker_x[3] - planner_x[3],
                tracker_x[4] - planner_x[4],
                tracker_x[5] - planner_x[5],
                tracker_x[6]
            ])

        # Populate output msg.
        msg = Control()
        msg.u = self.policy.OptimalControl(relative_x)
        msg.priority = 1.0
        return msg

    def TrackingBound(self):
        params = self.policy.TrackingBoundParams()
        res = TrackingBoundBoxResponse()
        res.x = params[0]
        res.y = params[1]
        res.z = params[2]
        return res

    def PlannerDynamics(self):
        max_speed = self.policy.PlannerMaxSpeed()
        res = KinematicPlannerDynamicsResponse()
        res.max_speed = max_speed
        res.min_speed = [-v for v in max_speed]
        return res
