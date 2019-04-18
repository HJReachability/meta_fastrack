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
// Defines the Tracker base class, which is templated on the following types:
// -- [Tracker] state (TS), control (TC)
//
// All trackers essentially work by subscribing to state and reference topics
// and periodically (on a timer) querying the value function for the optimal
// control, then publishing that optimal control. Trackers also must provide
// services that other nodes can use to access planner dynamics and bound
// parameters.
//
///////////////////////////////////////////////////////////////////////////////

// TO DO: Comment out everything that refers to values_

#ifndef META_PLANNER_TRACKER_H
#define META_Planner_TRACKER_H

// CHECK INCLUDES
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <fastrack_msgs/Control.h>
#include <fastrack_msgs/State.h>

#include <meta_planner_msgs/PlannerState.h>

#include <meta_planner_srvs/PlannerDynamics.h>
#include <meta_planner_srvs/TrackingBound.h>

#include <value_function/value_function.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

// CHECK TRACKING NAMESPACE
namespace meta {
namespace tracking {

// REMOVED V TEMPLATE
// reomved extra templates, use fastrack mesages state
template <typename TS, typename TC>
class Tracker : private fastrack::Uncopyable {
 public:
  ~Tracker() {}
  Tracker()
      : ready_(false),
        received_planner_x_(false),
        received_tracker_x_(false),
        initialized_(false) {}

  // Initialize from a ROS NodeHandle.
  bool Initialize(const ros::NodeHandle& n);

 private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Is the system ready? (same as inFlightCallback)
  void ReadyCallback(const std_msgs::Empty::ConstPtr& msg) { ready_ = true; }

  // Callback to update tracker/planner state.
  void TrackerStateCallback(const fastrack_msgs::State::ConstPtr& msg) {
    tracker_x_.FromRosPtr(msg);
    received_tracker_x_ = true;
  }
  void PlannerStateCallback(
      const meta_planner_msgs::PlannerState::ConstPtr& msg) {
    // planner state
    planner_x_.FromRos(msg->x);
    received_planner_x_ = true;

    // index for appropriate value function
    flattened_id_ =
        FromRowMajor(msg->previous_planner_id, msg->next_planner_id);
    received_flattened_id_ = true;
  }

  // Converts previous/next planner ids to flattened id for
  // value function matrix. row = previous, col = next
  size_t FromRowMajor(size_t row, size_t col) const {
    return num_planners_ * row + col;
  }

  // Service callbacks for tracking bound and planner parameters.
  bool TrackingBoundServer(
      typename meta_planner_srvs::TrackingBound::Request& req,
      typename meta_planner_srvs::TrackingBound::Response& res) {
    //  res = values_[FromRowMajor(
    //    req.previous_planner_id,req.next_planner_id)].TrackingBound().ToRos();
    return true;
  }

  bool PlannerDynamicsServer(
      typename meta_planner_srvs::PlannerDynamics::Request& req,
      typename meta_planner_srvs::PlannerDynamics::Response& res) {
    //  res = values_[FromRowMajor(
    //    req.planner_id,req.planner_id)].PlannerDynamics().ToRos();
    return true;
  }

  // Timer callback. Compute the optimal control and publish.
  inline void TimerCallback(const ros::TimerEvent& e) const {
    if (!ready_) return;

    if (!received_planner_x_ || !received_tracker_x_) {
      ROS_WARN_THROTTLE(1.0, "%s: Have not received planner/tracker state yet.",
                        name_.c_str());
      return;
    }

    // Publish bound.
    // values_[flattened_id_].TrackingBound().Visualize(bound_pub_,
    // planner_frame_);

    // Publish control.
    // control_pub_.publish(values_[flattened_id_].OptimalControl(
    //  tracker_x_, planner_x_).ToRos(
    //  values_[flattened_id_].Priority(tracker_x_, planner_x_)));
    //}
  }

  // number of planners
  size_t num_planners_;

  // Most recent tracker/planner states.
  TS tracker_x_;
  TS planner_x_;

  bool received_tracker_x_;
  bool received_planner_x_;

  // Previous and next planner IDs
  size_t flattened_id_;
  // size_t previous_planner_id_;
  // size_t next_planner_id_;

  bool received_flattened_id_;
  // bool received_previous_planner_id_;
  // bool received_next_planner_id_;

  // Value function.
  // V values_;
  // std::vector<ValueFunction::ConstPtr> values_;
  // need to define constptr for value function. check metaplanner

  // Planner frame of reference.
  std::string planner_frame_;

  // Publishers and subscribers.
  std::string ready_topic_;
  std::string tracker_state_topic_;
  std::string planner_state_topic_;
  std::string control_topic_;
  std::string bound_topic_;

  ros::Subscriber ready_sub_;
  ros::Subscriber tracker_state_sub_;
  ros::Subscriber planner_state_sub_;
  ros::Publisher control_pub_;
  ros::Publisher bound_pub_;

  // Services.
  std::string bound_name_;
  std::string planner_dynamics_name_;

  ros::ServiceServer bound_srv_;
  ros::ServiceServer planner_dynamics_srv_;

  // Timer.
  ros::Timer timer_;
  double time_step_;

  // Is the system ready for our control input?
  bool ready_;

  // Flag for whether this class has been initialized yet.
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
};  //\class Tracker

// ----------------------------- IMPLEMEMTATION ----------------------------- //

// Initialize from a ROS NodeHandle.
// template<typename V, typename TS, typename TC,
//         typename PS, typename SB, typename SP>
template <typename TS, typename TC>
bool Tracker<TS, TC>::Initialize(const ros::NodeHandle& n) {
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
  // NOTE: May want to change later based on how we initialize values
  // for ( ii = 0; ii < (num_planners_*num_planners_); ii = ii + 1 ) {
  //  if (!values_[ii].Initialize(n)) {
  //    ROS_ERROR("%s: Failed to initialize a value function.", name_.c_str());
  //    return false;
  //  }
  //}

  initialized_ = true;
  return true;
}

// Load parameters.
template <typename TS, typename TC>
// template<typename V, typename TS, typename TC,
//         typename PS, typename SB, typename SP>
bool Tracker<TS, TC>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/ready", ready_topic_)) return false;
  if (!nl.getParam("topic/tracker_state", tracker_state_topic_)) return false;
  if (!nl.getParam("topic/planner_state", planner_state_topic_)) return false;
  if (!nl.getParam("topic/control", control_topic_)) return false;
  if (!nl.getParam("vis/bound", bound_topic_)) return false;

  // Service names.
  if (!nl.getParam("srv/bound", bound_name_)) return false;
  if (!nl.getParam("srv/planner_dynamics", planner_dynamics_name_))
    return false;

  // Planner frame of reference.
  if (!nl.getParam("frames/planner", planner_frame_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  // Number of planners.
  int num_planners_temp;
  if (!nl.getParam("num_planners", num_planners_temp)) return false;
  num_planners_ = static_cast<size_t>(num_planners_temp);

  return true;
}

// Register callbacks.
template <typename TS, typename TC>
bool Tracker<TS, TC>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Services.
  bound_srv_ = nl.advertiseService(bound_name_.c_str(),
                                   &Tracker<TS, TC>::TrackingBoundServer, this);
  planner_dynamics_srv_ =
      nl.advertiseService(planner_dynamics_name_.c_str(),
                          &Tracker<TS, TC>::PlannerDynamicsServer, this);

  // Subscribers.
  ready_sub_ = nl.subscribe(ready_topic_.c_str(), 1,
                            &Tracker<TS, TC>::ReadyCallback, this);
  planner_state_sub_ =
      nl.subscribe(planner_state_topic_.c_str(), 1,
                   &Tracker<TS, TC>::PlannerStateCallback, this);
  tracker_state_sub_ =
      nl.subscribe(tracker_state_topic_.c_str(), 1,
                   &Tracker<TS, TC>::TrackerStateCallback, this);

  // Publishers.
  control_pub_ =
      nl.advertise<fastrack_msgs::Control>(control_topic_.c_str(), 1, false);
  bound_pub_ =
      nl.advertise<visualization_msgs::Marker>(bound_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_),
                          &Tracker<TS, TC>::TimerCallback, this);

  return true;
}

}  //\namespace tracking
}  //\namespace fastrack

#endif
