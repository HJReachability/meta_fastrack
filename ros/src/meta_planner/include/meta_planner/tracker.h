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

#ifndef META_PLANNER_TRACKER_H
#define META_PLANNER_TRACKER_H

#include <meta_planner/trajectory.h>
#include <meta_planner/ompl_planner.h>
#include <demo/balls_in_box.h>
#include <utils/types.h>
#include <utils/uncopyable.h>
#include <utils/message_interfacing.h>

#include <meta_planner_msgs/Trajectory.h>
#include <meta_planner_msgs/TrajectoryRequest.h>
#include <meta_planner_msgs/ControllerId.h>

#include <value_function/OptimalControl.h>
#include <value_function/Priority.h>

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

class Tracker : private Uncopyable {
public:
  explicit Tracker();
  ~Tracker();

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing state updates.
  void StateCallback(
    const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Callback for processing reference updates.
  void ReferenceCallback(
    const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Callback for processing controller ID updates.
  void ControllerIdCallback(
    const meta_planner_msgs::ControllerId::ConstPtr& msg);

  // Callback for applying tracking controller.
  void TimerCallback(const ros::TimerEvent& e);

  // Process in flight notifications.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }

  // Current state and reference.
  VectorXd state_;
  VectorXd reference_;

  // IDs of control/bound value functions.
  ValueFunctionId control_value_id_;
  ValueFunctionId bound_value_id_;

  // Spaces and dimensions.
  size_t control_dim_;
  size_t state_dim_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Service clients.
  ros::ServiceClient optimal_control_srv_;
  ros::ServiceClient priority_srv_;

  std::string optimal_control_name_;
  std::string priority_name_;

  // Publishers/subscribers and related topics.
  ros::Publisher control_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber reference_sub_;
  ros::Subscriber controller_id_sub_;
  ros::Subscriber in_flight_sub_;

  std::string control_topic_;
  std::string state_topic_;
  std::string reference_topic_;
  std::string controller_id_topic_;
  std::string in_flight_topic_;

  // Frames of reference for reading current pose from tf tree.
  std::string fixed_frame_id_;
  std::string tracker_frame_id_;
  std::string planner_frame_id_;

  // Are we in flight?
  bool in_flight_;

  // Has the state been updated.
  bool been_updated_;

  // Is this class initialized?
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
};

} //\namespace meta

#endif
