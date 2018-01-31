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
// Defines the TrajectoryInterpreter class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/trajectory_interpreter.h>
#include <crazyflie_utils/angles.h>

#include <stdlib.h>

namespace meta {

TrajectoryInterpreter::TrajectoryInterpreter()
  : in_flight_(false),
    been_updated_(false),
    original_goal_(true),
    initialized_(false) {}

TrajectoryInterpreter::~TrajectoryInterpreter() {}

// Initialize this class with all parameters and callbacks.
bool TrajectoryInterpreter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "trajectory_interpreter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set the initial state to zero.
  state_ = VectorXd::Zero(state_dim_);

  // Initialize trajectory to null.
  traj_ = nullptr;

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool TrajectoryInterpreter::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Control parameters.
  if (!nl.getParam("control/time_step", time_step_)) return false;

  int dimension = 1;
  if (!nl.getParam("control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  // Value parameters.
  if (!nl.getParam("max_runtime", max_meta_runtime_)) return false;

  // State space parameters.
  if (!nl.getParam("state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  // Service names.
  if (!nl.getParam("srv/tracking_bound", tracking_bound_name_)) return false;

  // Topics and frame ids.
  if (!nl.getParam("topics/state", state_topic_)) return false;
  if (!nl.getParam("topics/traj", traj_topic_)) return false;
  if (!nl.getParam("topics/reference", reference_topic_)) return false;
  if (!nl.getParam("topics/controller_id", controller_id_topic_))
    return false;

  if (!nl.getParam("topics/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("topics/trigger_replan", trigger_replan_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topics/vis/traj", traj_vis_topic_)) return false;
  if (!nl.getParam("topics/vis/tracking_bound", tracking_bound_topic_))
    return false;

  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/tracker", tracker_frame_id_)) return false;
  if (!nl.getParam("frames/planner", planner_frame_id_)) return false;

  // Start and goal.
  double goal_x, goal_y, goal_z;
  if (!nl.getParam("goal/x", goal_x)) return false;
  if (!nl.getParam("goal/y", goal_y)) return false;
  if (!nl.getParam("goal/z", goal_z)) return false;
  goal_ = Vector3d(goal_x, goal_y, goal_z);

  double start_x, start_y, start_z;
  if (!nl.getParam("start/x", start_x)) return false;
  if (!nl.getParam("start/y", start_y)) return false;
  if (!nl.getParam("start/z", start_z)) return false;
  start_ = Vector3d(start_x, start_y, start_z);

  return true;
}

// Register all callbacks and publishers.
bool TrajectoryInterpreter::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  traj_sub_ = nl.subscribe(
    traj_topic_.c_str(), 1, &TrajectoryInterpreter::TrajectoryCallback, this);

  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &TrajectoryInterpreter::StateCallback, this);

  trigger_replan_sub_ =
    nl.subscribe(trigger_replan_topic_.c_str(), 1,
                 &TrajectoryInterpreter::TriggerReplanCallback, this);

  in_flight_sub_ =
    nl.subscribe(in_flight_topic_.c_str(), 1,
                 &TrajectoryInterpreter::InFlightCallback, this);

  // Visualization publisher(s).
  traj_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    traj_vis_topic_.c_str(), 1, false);

  tracking_bound_pub_ = nl.advertise<visualization_msgs::Marker>(
    tracking_bound_topic_.c_str(), 1, false);

  // Actual publishers.
  reference_pub_ = nl.advertise<crazyflie_msgs::PositionVelocityStateStamped>(
    reference_topic_.c_str(), 1, false);

  controller_id_pub_ = nl.advertise<meta_planner_msgs::ControllerId>(
    controller_id_topic_.c_str(), 1, false);

  request_traj_pub_ = nl.advertise<meta_planner_msgs::TrajectoryRequest>(
    request_traj_topic_.c_str(), 1, false);

  // Service clients.
  tracking_bound_srv_ = nl.serviceClient<value_function::TrackingBoundBox>(
    tracking_bound_name_.c_str(), true);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_),
                          &TrajectoryInterpreter::TimerCallback, this);

  return true;
}

// Callback for processing trajectory updates.
void TrajectoryInterpreter::
TrajectoryCallback(const meta_planner_msgs::Trajectory::ConstPtr& msg) {
  traj_ = Trajectory::Create(msg);
}

// Callback for processing state updates.
void TrajectoryInterpreter::StateCallback(
  const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  state_(0) = msg->state.x;
  state_(1) = msg->state.y;
  state_(2) = msg->state.z;
  state_(3) = msg->state.x_dot;
  state_(4) = msg->state.y_dot;
  state_(5) = msg->state.z_dot;

  been_updated_ = true;
}

// Callback for when the MetaPlanner sees a new obstacle and wants the Tracker
// to hover and request a new trajectory.
void TrajectoryInterpreter::
TriggerReplanCallback(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("%s: Replan callback triggered.", name_.c_str());

  if (!in_flight_ || !been_updated_)
    return;

  // Set trajectory to be the remainder of this trajectory, then hovering
  // at the end for a while.
  Hover();

  // Request a new trajectory starting from the state we will be in after
  // the max_meta_runtime_ has elapsed.
  RequestNewTrajectory();
}

// Callback for applying tracking controller.
void TrajectoryInterpreter::TimerCallback(const ros::TimerEvent& e) {
  if (!in_flight_ || !been_updated_)
    return;

  ros::Time current_time = ros::Time::now();

  // (1) If current time is near the end of the current trajectory, just hover and
  //     post a request for a new trajectory.
  if (traj_ == nullptr ||
      current_time.toSec() > traj_->LastTime() - max_meta_runtime_) {
    ROS_WARN_THROTTLE(1.0, "%s: Nearing end of trajectory. Replanning.",
             name_.c_str());

    // Set trajectory to be the remainder of this trajectory, then hovering
    // at the end for a while.
    Hover();

    // Request a new trajectory starting from the state we will be in after
    // the max_meta_runtime_ has elapsed.
    RequestNewTrajectory();

    return;
  }

  const VectorXd planner_state = traj_->GetState(current_time.toSec());
  const VectorXd relative_state = state_ - planner_state;

  // HACK! Assuming state layout.
  const Vector3d planner_position(
    planner_state(0), planner_state(1), planner_state(2));

  // Publish planner state on tf.
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = fixed_frame_id_;
  transform_stamped.header.stamp = current_time;

  transform_stamped.child_frame_id = planner_frame_id_;

  transform_stamped.transform.translation.x = planner_position(0);
  transform_stamped.transform.translation.y = planner_position(1);
  transform_stamped.transform.translation.z = planner_position(2);

  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;

  br_.sendTransform(transform_stamped);

  // (2) Get corresponding control and bound value function.
  const ValueFunctionId control_value_id =
    traj_->GetControlValueFunction(current_time.toSec());
  const ValueFunctionId bound_value_id =
    traj_->GetBoundValueFunction(current_time.toSec());

  // Publish planner position to the reference topic.
  // HACK! Assuming planner state order.
  crazyflie_msgs::PositionVelocityStateStamped reference;
  reference.header.stamp = current_time;

  reference.state.x = planner_position(0);
  reference.state.y = planner_position(1);
  reference.state.z = planner_position(2);

  reference.state.x_dot = planner_state(3);
  reference.state.y_dot = planner_state(4);
  reference.state.z_dot = planner_state(5);

  reference_pub_.publish(reference);

  meta_planner_msgs::ControllerId controller_id;
  controller_id.control_value_function_id = control_value_id;
  controller_id.bound_value_function_id = bound_value_id;

  controller_id_pub_.publish(controller_id);

  // Visualize the tracking bound.
  visualization_msgs::Marker tracking_bound_marker;
  tracking_bound_marker.ns = "bound";
  tracking_bound_marker.header.frame_id = planner_frame_id_;
  tracking_bound_marker.header.stamp = current_time;
  tracking_bound_marker.id = 0;
  tracking_bound_marker.type = visualization_msgs::Marker::CUBE;
  tracking_bound_marker.action = visualization_msgs::Marker::ADD;

  tracking_bound_marker.scale.x = 0.0;
  tracking_bound_marker.scale.y = 0.0;
  tracking_bound_marker.scale.z = 0.0;

  value_function::TrackingBoundBox b;

  if (!tracking_bound_srv_) {
    ROS_WARN("%s: Tracking bound server disconnected.", name_.c_str());
    ros::NodeHandle nl;
    tracking_bound_srv_ = nl.serviceClient<value_function::TrackingBoundBox>(
      tracking_bound_name_.c_str(), true);
  } else {
   // value_function::TrackingBoundBox b;
    b.request.id = bound_value_id;
    if (!tracking_bound_srv_.call(b))
      ROS_ERROR("%s: Tracking bound server error.", name_.c_str());
    else {
      tracking_bound_marker.scale.x = 2.0 * b.response.x;
      tracking_bound_marker.scale.y = 2.0 * b.response.y;
      tracking_bound_marker.scale.z = 2.0 * b.response.z;
    }
  }

  tracking_bound_marker.color.a = 0.3;
  tracking_bound_marker.color.r = 0.5;
  tracking_bound_marker.color.g = 0.1;
  tracking_bound_marker.color.b = 0.5;

  tracking_bound_pub_.publish(tracking_bound_marker);

  // Visualize trajectory.
  traj_->Visualize(traj_vis_pub_, fixed_frame_id_);

  // Check if we're near the current goal, and if so switch the goal
  // and issue a replan request.
  const Vector3d current_goal = (original_goal_) ? goal_ : start_;

  std::printf("Tracking bound was (%f, %f, %f).\n", b.response.x, b.response.y, b.response.z);

  // HACK! Assuming state layout.
  if (std::abs(state_(0) - current_goal(0)) <= b.response.x &&
      std::abs(state_(1) - current_goal(1)) <= b.response.y &&
      std::abs(state_(2) - current_goal(2)) <= b.response.z) {
    std::cout << "Flipping goal points." << std::endl;
    original_goal_ = !original_goal_;
    Hover();
    RequestNewTrajectory();
  }
}

// Request a new trajectory from the meta planner.
void TrajectoryInterpreter::RequestNewTrajectory() const {
  if (traj_ == nullptr) {
    ROS_ERROR("%s: Tried to request new trajectory without knowing.",
              name_.c_str());
    return;
  }

  if (original_goal_){
    ROS_INFO("%s: Requesting a new trajectory to goal.", name_.c_str());  
  }else{
    ROS_INFO("%s: Requesting a new trajectory to start.", name_.c_str());
  }
  

  // Determine time and state where we will receive the new trajectory.
  // This is when/where the new trajectory should start from.
  const double start_time = ros::Time::now().toSec() + max_meta_runtime_;
  const VectorXd start_state = traj_->GetState(start_time);

  // Populate request.
  // HACK! Assuming state layout.
  meta_planner_msgs::TrajectoryRequest msg;
  msg.start_time = start_time;
  msg.start_position.x = start_state(0);
  msg.start_position.y = start_state(1);
  msg.start_position.z = start_state(2);

  if (original_goal_) {
    msg.stop_position.x = goal_(0);
    msg.stop_position.y = goal_(1);
    msg.stop_position.z = goal_(2);
  } else {
    msg.stop_position.x = start_(0);
    msg.stop_position.y = start_(1);
    msg.stop_position.z = start_(2);
  }

  request_traj_pub_.publish(msg);
}

// Set the trajectory to continue the existing trajectory, then hover at
// the end. If no current trajectory exists, simply hover at the current state.
void TrajectoryInterpreter::Hover() {
  ROS_INFO("%s: Setting a hover trajectory.", name_.c_str());

  // Get the current time.
  const double now = ros::Time::now().toSec();

  // Catch null trajectory, which should only occur on startup.
  // Hover at the current trajectory for max_meta_runtime_ + some small amount.
  // Set the value function for this trajectory to be the most aggressive planner
  // so that it has the largest error bound.
  if (traj_ == nullptr) {
    ROS_INFO("%s: No existing trajectory. Hovering in place.", name_.c_str());
    traj_ = Trajectory::Create();

    // Get a zero-velocity version of the current state.
    // HACK! Assuming state layout.
    VectorXd hover_state = state_;
    hover_state(3) = 0.0;
    hover_state(4) = 0.0;
    hover_state(5) = 0.0;

    traj_->Add(now, hover_state, 0, 0);
    traj_->Add(now + max_meta_runtime_ + 50.0, hover_state, 0, 0);
    return;
  }

  ROS_INFO("%s: Hovering at the end of the current trajectory.", name_.c_str());

  // Non-null current trajectory.
  // Copy over the remainder of the current trajectory.
  Trajectory::Ptr hover = Trajectory::Create(traj_, now);

  // Get the last state and make sure it has zero velocity.
  // HACK! Assuming state layout.
  VectorXd last_state = hover->LastState();
  last_state(3) = 0.0;
  last_state(4) = 0.0;
  last_state(5) = 0.0;

  // Hover at the last state.
  // HACK! Assuming can hover using last value function. Is this true?
  hover->Add(hover->LastTime() + max_meta_runtime_ + 10.0,
             last_state,
             hover->LastControlValueFunction(),
             hover->LastControlValueFunction());

  traj_ = hover;
}

} //\namespace meta
