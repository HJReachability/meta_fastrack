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
// Defines the Tracker class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/tracker.h>
#include <crazyflie_utils/angles.h>

#include <stdlib.h>

namespace meta {

Tracker::Tracker()
  : in_flight_(false),
    been_updated_(false),
    initialized_(false) {}

Tracker::~Tracker() {}

// Initialize this class with all parameters and callbacks.
bool Tracker::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "tracker");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set the initial state and reference to zero.
  state_ = VectorXd::Zero(state_dim_);
  reference_ = VectorXd::Zero(state_dim_);

  // Start control and bound values at most/least conservative.
  control_value_id_ = 0;
  bound_value_id_ = 0;

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Tracker::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Control parameters.
  if (!nl.getParam("control/time_step", time_step_)) return false;

  int dimension = 1;
  if (!nl.getParam("control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  // State space parameters.
  if (!nl.getParam("state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  // Service names.
  if (!nl.getParam("srv/optimal_control", optimal_control_name_))
    return false;
  if (!nl.getParam("srv/priority", priority_name_))
    return false;

  // Topics and frame ids.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topics/state", state_topic_)) return false;
  if (!nl.getParam("topics/reference", reference_topic_)) return false;
  if (!nl.getParam("topics/controller_id", controller_id_topic_))
    return false;

  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/tracker", tracker_frame_id_)) return false;
  if (!nl.getParam("frames/planner", planner_frame_id_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool Tracker::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &Tracker::StateCallback, this);

  reference_sub_ = nl.subscribe(
    reference_topic_.c_str(), 1, &Tracker::ReferenceCallback, this);

  controller_id_sub_ = nl.subscribe(
    controller_id_topic_.c_str(), 1, &Tracker::ControllerIdCallback, this);

  in_flight_sub_ = nl.subscribe(
    in_flight_topic_.c_str(), 1, &Tracker::InFlightCallback, this);

  // Actual publishers.
  control_pub_ = nl.advertise<crazyflie_msgs::NoYawControlStamped>(
    control_topic_.c_str(), 1, false);

  // Service clients.
  optimal_control_srv_ = nl.serviceClient<value_function::OptimalControl>(
    optimal_control_name_.c_str(), true);

  priority_srv_ = nl.serviceClient<value_function::Priority>(
    priority_name_.c_str(), true);

  // Timer.
  timer_ =
    nl.createTimer(ros::Duration(time_step_), &Tracker::TimerCallback, this);

  return true;
}

// Callback for processing state updates.
void Tracker::
StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  state_(0) = msg->state.x;
  state_(1) = msg->state.y;
  state_(2) = msg->state.z;
  state_(3) = msg->state.x_dot;
  state_(4) = msg->state.y_dot;
  state_(5) = msg->state.z_dot;

  been_updated_ = true;
}

// Callback for processing state updates.
void Tracker::ReferenceCallback(
  const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  reference_(0) = msg->state.x;
  reference_(1) = msg->state.y;
  reference_(2) = msg->state.z;
  reference_(3) = msg->state.x_dot;
  reference_(4) = msg->state.y_dot;
  reference_(5) = msg->state.z_dot;
}

// Callback for processing state updates.
void Tracker::ControllerIdCallback(
  const meta_planner_msgs::ControllerId::ConstPtr& msg) {
  control_value_id_ = msg->control_value_function_id;
  bound_value_id_ = msg->bound_value_function_id;
}

// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  if (!in_flight_ || !been_updated_)
    return;

  // HACK! Assuming state layout.
  const VectorXd relative_state = state_ - reference_;
  const Vector3d planner_position(reference_(0), reference_(1), reference_(2));

  // (1) Get priority.
  if (!priority_srv_) {
    ROS_WARN("%s: Priority server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    priority_srv_ = nl.serviceClient<value_function::Priority>(
      priority_name_.c_str(), true);

    return;
  }

  double priority = 0.0;

  value_function::Priority p;
  p.request.id = control_value_id_;
  p.request.state = utils::PackState(relative_state);
  if (!priority_srv_.call(p))
    ROS_ERROR("%s: Error calling priority server.", name_.c_str());
  else
    priority = p.response.priority;

  // (2) Get optimal control.
  if (!optimal_control_srv_) {
    ROS_WARN("%s: Optimal control server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    optimal_control_srv_ = nl.serviceClient<value_function::OptimalControl>(
      optimal_control_name_.c_str(), true);

    return;
  }

  VectorXd optimal_control(control_dim_);

  value_function::OptimalControl c;
  c.request.id = control_value_id_;
  c.request.state = utils::PackState(relative_state);
  if (!optimal_control_srv_.call(c))
    ROS_ERROR("%s: Error calling optimal control server.", name_.c_str());
  else
    optimal_control = utils::Unpack(c.response.control);

  // (3) Publish optimal control with priority in (0, 1).
  crazyflie_msgs::NoYawControlStamped control_msg;
  control_msg.header.stamp = ros::Time::now();

  // NOTE! Remember, control is assumed to be [pitch, roll, thrust].
  control_msg.control.pitch =
    crazyflie_utils::angles::WrapAngleRadians(optimal_control(0));
  control_msg.control.roll =
    crazyflie_utils::angles::WrapAngleRadians(optimal_control(1));
  control_msg.control.thrust = optimal_control(2);
  control_msg.control.priority = priority;

  control_pub_.publish(control_msg);
}

} //\namespace meta
