/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
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
// Defines the PlannerManager class, which listens for new trajectory
// messages and, on a timer, repeatedly queries the current trajectory and
// publishes the corresponding reference.
//
// The PlannerManager is also responsible for requesting new plans.
// This base class only calls the planner once upon takeoff; as needs will vary
// derived classes may add further functionality such as receding
// horizon planning.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/planning/planner_manager.h>
#include <meta_planner/trajectory/trajectory.h>

#include <meta_planner_msgs/ReplanRequest.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace meta {
namespace planning {
using meta_planner::trajectory::Trajectory;

bool PlannerManager::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PlannerManager");

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

  initialized_ = true;
  return true;
}

bool PlannerManager::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/ready", ready_topic_)) return false;
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/ref", ref_topic_)) return false;
  if (!nl.getParam("topic/original_fastrack_ref", original_fastrack_ref_topic_))
    return false;
  if (!nl.getParam("topic/replan_request", replan_request_topic_)) return false;
  if (!nl.getParam("topic/updated_env", updated_env_topic_)) return false;
  if (!nl.getParam("vis/traj", traj_vis_topic_)) return false;
  if (!nl.getParam("vis/goal", goal_topic_)) return false;

  // Frames.
  if (!nl.getParam("frame/fixed", fixed_frame_)) return false;
  if (!nl.getParam("frame/planner", planner_frame_)) return false;

  // Time step.
  if (!nl.getParam("time_step", time_step_)) return false;

  // Planner runtime, start, and goal.
  if (!nl.getParam("planner_runtime", planner_runtime_)) return false;

  if (!nl.getParam("start", start_.x)) return false;
  if (!nl.getParam("goal", goal_.x)) return false;

  return true;
}

bool PlannerManager::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  ready_sub_ = nl.subscribe(ready_topic_.c_str(), 1,
                            &PlannerManager::ReadyCallback, this);

  traj_sub_ = nl.subscribe(traj_topic_.c_str(), 1,
                           &PlannerManager::TrajectoryCallback, this);

  updated_env_sub_ =
      nl.subscribe(updated_env_topic_.c_str(), 1,
                   &PlannerManager::UpdatedEnvironmentCallback, this);

  // Publishers.
  original_fastrack_ref_pub_ = nl.advertise<fastrack_msgs::State>(
      original_fastrack_ref_topic_.c_str(), 1, false);

  ref_pub_ = nl.advertise<meta_planner_msgs::PlannerState>(ref_topic_.c_str(),
                                                           1, false);

  replan_request_pub_ = nl.advertise<meta_planner_msgs::ReplanRequest>(
      replan_request_topic_.c_str(), 1, false);

  goal_pub_ =
      nl.advertise<visualization_msgs::Marker>(goal_topic_.c_str(), 1, false);

  traj_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
      traj_vis_topic_.c_str(), 1, false);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(time_step_),
                          &PlannerManager::TimerCallback, this);

  return true;
}

void PlannerManager::MaybeRequestTrajectory() {
  // Publish marker at goal location.
  VisualizeGoal();

  if (!ready_ || waiting_for_traj_) {
    return;
  }

  // Set start and goal states.
  meta_planner_msgs::ReplanRequest msg;
  msg.start = start_;
  msg.goal = goal_;
  msg.initial_planner_id = 0;

  // Set start time.
  msg.start_time = ros::Time::now().toSec() + planner_runtime_;

  // Reset start state for future state if we have a current trajectory.
  if (traj_.Size() > 0) {
    // Catch trajectory that's too short.
    if (traj_.LastTime() < msg.start_time) {
      ROS_ERROR("%s: Current trajectory is too short. Cannot interpolate.",
                name_.c_str());
      msg.start = traj_.LastState();
    } else {
      const meta_planner_msgs::PlannerState interpolated =
          traj_.Interpolate(msg.start_time);
      msg.start = interpolated.previous_planner_state;
      msg.initial_planner_id = interpolated.previous_planner_id;
    }
  }

  // Publish request and set flag.
  replan_request_pub_.publish(msg);
  waiting_for_traj_ = true;
  serviced_updated_env_ = true;
}

void PlannerManager::TimerCallback(const ros::TimerEvent& e) {
  if (!ready_) return;

  if (traj_.Size() == 0) {
    MaybeRequestTrajectory();
    return;
  } else if (waiting_for_traj_) {
    ROS_WARN_THROTTLE(1.0, "%s: Waiting for trajectory.", name_.c_str());
  } else if (!serviced_updated_env_) {
    ROS_INFO_THROTTLE(1.0, "%s: Servicing old updated environment callback.",
                      name_.c_str());
    MaybeRequestTrajectory();
  }

  // Interpolate the current trajectory.
  const meta_planner_msgs::PlannerState planner_x =
      traj_.Interpolate(ros::Time::now().toSec());

  // Convert to ROS msg and publish.
  original_fastrack_ref_pub_.publish(planner_x.next_planner_state);
  ref_pub_.publish(planner_x);

  // Broadcast transform.
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = fixed_frame_;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = planner_frame_;

  tf.transform.translation.x = planner_x.position.x;
  tf.transform.translation.y = planner_x.position.y;
  tf.transform.translation.z = planner_x.position.z;
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;

  tf_broadcaster_.sendTransform(tf);
}

void PlannerManager::TrajectoryCallback(
    const meta_planner_msgs::Trajectory::ConstPtr& msg) {
  waiting_for_traj_ = false;

  // Catch failure (empty msg).
  if (msg->states.empty() || msg->times.empty()) {
    ROS_WARN_THROTTLE(1.0, "%s: Received empty trajectory.", name_.c_str());
    return;
  }

  // Update current trajectory and visualize.
  traj_ = Trajectory(*msg);
  traj_.Visualize(traj_vis_pub_, fixed_frame_);
}

void PlannerManager::UpdatedEnvironmentCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  serviced_updated_env_ = false;
  // NOTE: not going to request a new trajector unless the timer fires, because
  // otherwise we'll be firing off trajectory requests *with each planner
  // environment update*.
  //  MaybeRequestTrajectory();
}

void PlannerManager::VisualizeGoal() const {
  // Set up sphere marker.
  visualization_msgs::Marker sphere;
  sphere.ns = "sphere";
  sphere.header.frame_id = fixed_frame_;
  sphere.header.stamp = ros::Time::now();
  sphere.id = 0;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.color.a = 1.0;
  sphere.color.r = 0.3;
  sphere.color.g = 0.7;
  sphere.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  sphere.scale.x = 0.1;
  center.x = goal_.x[0];

  sphere.scale.y = 0.1;
  center.y = goal_.x[1];

  sphere.scale.z = 0.1;
  center.z = goal_.x[2];

  sphere.pose.position = center;
  sphere.pose.orientation.x = 0.0;
  sphere.pose.orientation.y = 0.0;
  sphere.pose.orientation.z = 0.0;
  sphere.pose.orientation.w = 1.0;

  goal_pub_.publish(sphere);

  return;
}

}  //\namespace planning
}  //\namespace meta
