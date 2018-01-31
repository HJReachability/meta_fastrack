/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
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
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the TrajectoryInterpreter class, which listens for new trajectory
// messages and, on a timer, repeatedly queries the current trajectory and
// publishes the corresponding reference.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_TRAJECTORY_INTERPRETER_H
#define META_PLANNER_TRAJECTORY_INTERPRETER_H

#include <meta_planner/trajectory.h>
#include <utils/types.h>
#include <utils/uncopyable.h>
#include <utils/message_interfacing.h>

#include <value_function/TrackingBoundBox.h>

#include <meta_planner_msgs/Trajectory.h>
#include <meta_planner_msgs/TrajectoryRequest.h>
#include <meta_planner_msgs/ControllerId.h>

#include <crazyflie_msgs/PositionVelocityStateStamped.h>
#include <crazyflie_msgs/ControlStamped.h>
#include <crazyflie_msgs/NoYawControlStamped.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <math.h>

namespace meta {

class TrajectoryInterpreter : private Uncopyable {
public:
  explicit TrajectoryInterpreter();
  ~TrajectoryInterpreter();

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing trajectory updates.
  void TrajectoryCallback(const meta_planner_msgs::Trajectory::ConstPtr& msg);

  // Callback for processing state updates.
  void StateCallback(
    const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Callback for applying tracking controller.
  void TimerCallback(const ros::TimerEvent& e);

  // Request a new trajectory from the meta planner.
  void RequestNewTrajectory() const;

  // Callback for when the MetaPlanner sees a new obstacle and wants
  // the Tracker to hover and request a new trajectory.
  void TriggerReplanCallback(const std_msgs::Empty::ConstPtr& msg);

  // Process in flight notifications.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }

  // Send a hover control.
  void Hover();

  // Current state and trajectory.
  VectorXd state_;
  Trajectory::Ptr traj_;

  // Maximum runtime for meta planner.
  double max_meta_runtime_;

  // TF interfacing.
  tf2_ros::TransformBroadcaster br_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Spaces and dimensions.
  size_t control_dim_;
  size_t state_dim_;

  // Servers and names.
  ros::ServiceClient tracking_bound_srv_;
  std::string tracking_bound_name_;

  // Publishers/subscribers and related topics.
  ros::Publisher tracking_bound_pub_;
  ros::Publisher reference_pub_;
  ros::Publisher controller_id_pub_;
  ros::Publisher request_traj_pub_;
  ros::Publisher traj_vis_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber trigger_replan_sub_;
  ros::Subscriber in_flight_sub_;

  std::string tracking_bound_topic_;
  std::string reference_topic_;
  std::string controller_id_topic_;
  std::string request_traj_topic_;
  std::string traj_vis_topic_;
  std::string traj_topic_;
  std::string state_topic_;
  std::string trigger_replan_topic_;
  std::string in_flight_topic_;

  // Frames of reference for reading current pose from tf tree.
  std::string fixed_frame_id_;
  std::string tracker_frame_id_;
  std::string planner_frame_id_;

  // Start and goal positions, and bool for whether we're going toward
  // the original goal or the start.
  Vector3d start_, goal_;
  bool original_goal_;

  // Has the state been updated.
  bool been_updated_;

  // Are we in flight?
  bool in_flight_;

  // Is this class initialized?
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
};

} //\namespace meta

#endif
