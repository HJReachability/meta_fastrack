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
// Defines the Simulator class. Holds a BallsInBox environment and sends
// simulated sensor measurements consisting of detected balls in range.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DEMO_SENSOR_H
#define DEMO_SENSOR_H

#include <demo/balls_in_box.h>
#include <demo/lanterns_in_box.h>
#include <value_function/near_hover_quad_no_yaw.h>
#include <utils/types.h>
#include <utils/uncopyable.h>

#include <meta_planner_msgs/SensorMeasurement.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace meta {

class Sensor : private Uncopyable {
public:
  explicit Sensor();
  ~Sensor();

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Update in flight status.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }

  // Timer callback for generating sensor measurements and updating
  // state based on last received control signal.
  void TimerCallback(const ros::TimerEvent& e);

  // Sensor radius.
  double sensor_radius_;

  // State space.
  BallsInBox::Ptr space_;
  unsigned int seed_;
  size_t num_obstacles_;
  size_t state_dim_;
  size_t control_dim_;

  NearHoverQuadNoYaw::ConstPtr dynamics_;

  std::vector<double> state_lower_;
  std::vector<double> state_upper_;

  // Max/min obstacle size.
  double max_obstacle_radius_;
  double min_obstacle_radius_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Publishers/subscribers and related topics.
  ros::Publisher sensor_radius_pub_;
  ros::Publisher environment_pub_;
  ros::Publisher sensor_pub_;
  ros::Subscriber in_flight_sub_;

  std::string sensor_radius_topic_;
  std::string environment_topic_;
  std::string sensor_topic_;
  std::string in_flight_topic_;

  /// Don't start sensing until we are in flight.
  bool in_flight_;

  // Frames of reference for reading current pose from tf tree.
  std::string fixed_frame_id_;
  std::string robot_frame_id_;

  // TF stuff.
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  // Is this class initialized?
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
};

} //\namespace meta

#endif
