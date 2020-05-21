/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Sylvia Herbert ( sylvia.herbert@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the Tracker class.
//
// All trackers essentially work by subscribing to state and reference topics
// and periodically (on a timer) querying the value function for the optimal
// control, then publishing that optimal control. Trackers also must provide
// services that other nodes can use to access planner dynamics and bound
// parameters.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/tracking/tracker.h>
#include <meta_planner/value/matlab_value_function.h>

#include <fastrack_msgs/Control.h>
#include <fastrack_msgs/State.h>
#include <fastrack_srvs/KinematicPlannerDynamics.h>
#include <fastrack_srvs/KinematicPlannerDynamicsRequest.h>
#include <fastrack_srvs/KinematicPlannerDynamicsResponse.h>
#include <fastrack_srvs/TrackingBoundBox.h>
#include <fastrack_srvs/TrackingBoundBoxRequest.h>
#include <fastrack_srvs/TrackingBoundBoxResponse.h>

#include <meta_planner_msgs/PlannerState.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace meta_planner {
namespace tracking {

// Initialize from a ROS NodeHandle.
bool Tracker::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Tracker");

  // Load parameters.
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  // Register callbacks.
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize value function.
  for (size_t ii = 0; ii < num_planners_ * num_planners_; ii++) {
    values_.push_back(value::MatlabValueFunction());
    if (!values_.back().InitializeFromMatFile(mat_files_[ii])) {
      ROS_ERROR("%s: Failed to initialize value function %zu.", name_.c_str(),
                ii);
      return false;
    }
  }

  // Start the timer.
  timer_.start();

  ROS_INFO("%s: Finished initializing tracker. All value functions loaded.",
           name_.c_str());

  initialized_ = true;
  return true;
}

bool Tracker::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/ready", ready_topic_)) return false;
  if (!nl.getParam("topic/tracker_state", tracker_state_topic_)) return false;
  if (!nl.getParam("topic/planner_state", planner_state_topic_)) return false;
  if (!nl.getParam("topic/control", control_topic_)) return false;
  if (!nl.getParam("vis/bound", bound_topic_)) return false;

  // Service names.
  if (!nl.getParam("srv/bound", bound_names_)) return false;
  if (!nl.getParam("srv/planner_dynamics", planner_dynamics_names_))
    return false;

  // Planner frame of reference.
  if (!nl.getParam("frames/planner", planner_frame_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  // Number of planners.
  int num_planners_temp;
  if (!nl.getParam("num_planners", num_planners_temp)) return false;
  num_planners_ = static_cast<size_t>(num_planners_temp);

  // Mat files for loading value functions.
  if (!nl.getParam("value/mat_files", mat_files_)) return false;
  ROS_ASSERT(mat_files_.size() == num_planners_ * num_planners_);

  return true;
}

bool Tracker::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  ready_sub_ =
      nl.subscribe(ready_topic_.c_str(), 1, &Tracker::ReadyCallback, this);
  planner_state_sub_ = nl.subscribe(planner_state_topic_.c_str(), 1,
                                    &Tracker::PlannerStateCallback, this);
  tracker_state_sub_ = nl.subscribe(tracker_state_topic_.c_str(), 1,
                                    &Tracker::TrackerStateCallback, this);

  // Publishers.
  control_pub_ =
      nl.advertise<fastrack_msgs::Control>(control_topic_.c_str(), 1, false);
  bound_pub_ =
      nl.advertise<visualization_msgs::Marker>(bound_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_), &Tracker::TimerCallback,
                          this, false);

  // Services as lambdas.
  for (size_t ii = 0; ii < bound_names_.size(); ii++) {
    const std::string& bound_srv_name = bound_names_[ii];

    // Generate a lambda function for this callback.
    boost::function<bool(fastrack_srvs::TrackingBoundBox::Request&,
                         fastrack_srvs::TrackingBoundBox::Response&)>
        bound_callback = [=](fastrack_srvs::TrackingBoundBox::Request& req,
                             fastrack_srvs::TrackingBoundBox::Response& res) {
          res = values_[ii].TrackingBound().ToRos();

          return true;
        };  // callback

    // Create a new service with this callback.
    bound_srvs_.push_back(
        nl.advertiseService(bound_srv_name.c_str(), bound_callback));

    // Same thing, but for planner dynamics.
    const std::string& dynamics_srv_name = planner_dynamics_names_[ii];

    // Generate a lambda function for this callback.
    boost::function<bool(fastrack_srvs::KinematicPlannerDynamics::Request&,
                         fastrack_srvs::KinematicPlannerDynamics::Response&)>
        dynamics_callback =
            [=](fastrack_srvs::KinematicPlannerDynamics::Request& req,
                fastrack_srvs::KinematicPlannerDynamics::Response& res) {
              res = values_[ii].PlannerDynamics();

              return true;
            };  // callback

    // Create a new service with this callback.
    planner_dynamics_srvs_.push_back(
        nl.advertiseService(dynamics_srv_name.c_str(), dynamics_callback));
  }

  return true;
}

}  //\namespace tracking
}  //\namespace meta_planner
