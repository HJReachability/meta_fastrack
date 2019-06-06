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

#ifndef META_PLANNER_TRACKER_H
#define META_PLANNER_TRACKER_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/value/matlab_value_function.h>

#include <fastrack_msgs/Control.h>
#include <fastrack_msgs/State.h>
#include <meta_planner_msgs/PlannerState.h>

#include <meta_planner_srvs/PlannerDynamics.h>
#include <meta_planner_srvs/PlannerDynamicsRequest.h>
#include <meta_planner_srvs/PlannerDynamicsResponse.h>
#include <meta_planner_srvs/TrackingBound.h>
#include <meta_planner_srvs/TrackingBoundRequest.h>
#include <meta_planner_srvs/TrackingBoundResponse.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

namespace meta_planner {
namespace tracking {

class Tracker : private fastrack::Uncopyable {
 public:
  ~Tracker() {}
  Tracker() : ready_(false), initialized_(false) {}

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
    tracker_x_ = msg;
  }
  void PlannerStateCallback(
      const meta_planner_msgs::PlannerState::ConstPtr& msg) {
    planner_x_ = msg;
    flattened_value_id_ =
        FromRowMajor(msg->previous_planner_id, msg->next_planner_id);
  }

  // Converts previous/next planner ids to flattened id for
  // value function matrix. row = previous, col = next
  size_t FromRowMajor(size_t row, size_t col) const {
    return num_planners_ * row + col;
  }

  // Service callbacks for tracking bound and planner parameters.
  bool TrackingBoundServer(meta_planner_srvs::TrackingBound::Request& req,
                           meta_planner_srvs::TrackingBound::Response& res) {
    res.params =
        values_[FromRowMajor(req.previous_planner_id, req.next_planner_id)]
            .TrackingBoundParams();

    return true;
  }

  bool PlannerDynamicsServer(
      meta_planner_srvs::PlannerDynamics::Request& req,
      meta_planner_srvs::PlannerDynamics::Response& res) {
    res.params = values_[FromRowMajor(req.planner_id, req.planner_id)]
                     .PlannerDynamicsParams();
    return true;
  }

  // Timer callback. Compute the optimal control and publish.
  inline void TimerCallback(const ros::TimerEvent& e) const {
    if (!ready_) return;

    if (!planner_x_.get() || !tracker_x_.get()) {
      ROS_WARN_THROTTLE(1.0, "%s: Have not received planner/tracker state yet.",
                        name_.c_str());
      return;
    }

    // Publish bound.
    values_[flattened_value_id_].TrackingBound().Visualize(bound_pub_,
                                                           planner_frame_);

    // Publish control.
    // NOTE: sending previous planner state because previous and next are
    // identical when interpolated.
    control_pub_.publish(values_[flattened_value_id_].OptimalControl(
        *tracker_x_, planner_x_->previous_planner_state));
  }

  // number of planners
  size_t num_planners_;

  // Most recent tracker/planner states.
  meta_planner_msgs::PlannerState::ConstPtr planner_x_;
  fastrack_msgs::State::ConstPtr tracker_x_;

  // Previous and next planner IDs
  size_t flattened_value_id_;

  // Value functions and list of mat files.
  std::vector<meta_planner::value::MatlabValueFunction> values_;
  std::vector<std::string> mat_files_;

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

}  //\namespace tracking
}  //\namespace meta_planner

#endif
