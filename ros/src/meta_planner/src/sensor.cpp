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

#include <demo/sensor.h>
#include <random>

namespace meta {

Sensor::Sensor()
  : tf_listener_(tf_buffer_),
    in_flight_(false),
    initialized_(false) {}

Sensor::~Sensor() {}

// Initialize this class with all parameters and callbacks.
bool Sensor::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "sensor");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize state space.
  space_ = BallsInBox::Create();
  if (!space_->Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize BallsInBox.", name_.c_str());
    return false;
  }

  // Dynamics with dummy control bounds. We only need puncturing functionality.
  dynamics_ = NearHoverQuadNoYaw::Create(VectorXd::Zero(control_dim_),
                                         VectorXd::Zero(control_dim_));

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(dynamics_->Puncture(state_lower_vec),
                    dynamics_->Puncture(state_upper_vec));

  space_->Seed(seed_);

  // Add obstacles.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> uniform_radius(
    min_obstacle_radius_, max_obstacle_radius_);

  // Add an obstacle with a random radius at a random location.
  for (size_t ii = 0; ii < num_obstacles_; ii++)
    space_->AddObstacle(space_->Sample(), uniform_radius(rng));

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Sensor::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Sensor radius.
  if (!nl.getParam("sensor/sensor_radius", sensor_radius_)) return false;

  // Random seed.
  int seed = 0;
  if (!nl.getParam("random/seed", seed)) return false;
  seed_ = static_cast<unsigned int>(seed);

  // Number of obstacles.
  int num_obstacles = 1;
  if (!nl.getParam("sensor/num_obstacles", num_obstacles)) return false;
  num_obstacles_ = static_cast<size_t>(num_obstacles);

  // Obstacle size.
  if (!nl.getParam("sensor/max_obstacle_radius", max_obstacle_radius_)) return false;
  if (!nl.getParam("sensor/min_obstacle_radius", min_obstacle_radius_)) return false;

  // Time step.
  if (!nl.getParam("sensor/time_step", time_step_)) return false;

  // State space parameters.
  int dimension = 1;
  if (!nl.getParam("control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("state/upper", state_upper_)) return false;
  if (!nl.getParam("state/lower", state_lower_)) return false;

  // Topics and frame ids.
  if (!nl.getParam("topics/sensor", sensor_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topics/vis/sensor_radius", sensor_radius_topic_)) return false;
  if (!nl.getParam("topics/vis/true_environment", environment_topic_)) return false;

  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/tracker", robot_frame_id_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool Sensor::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

   // Publishers.
  environment_pub_ = nl.advertise<visualization_msgs::Marker>(
    environment_topic_.c_str(), 1, false);

  sensor_radius_pub_ = nl.advertise<visualization_msgs::Marker>(
    sensor_radius_topic_.c_str(), 1, false);

  sensor_pub_ = nl.advertise<meta_planner_msgs::SensorMeasurement>(
    sensor_topic_.c_str(), 1, false);

  // Subscriber.
  in_flight_sub_ = nl.subscribe(
    in_flight_topic_.c_str(), 1, &Sensor::InFlightCallback, this);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Sensor::TimerCallback, this);

  return true;
}

// Timer callback for generating sensor measurements and updating
// state based on last received control signal.
void Sensor::TimerCallback(const ros::TimerEvent& e) {
  if (!in_flight_)
    return;

  // Read position from TF.
  const ros::Time right_now = ros::Time::now();

  // Get the current transform from tf.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), robot_frame_id_.c_str(), ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    return;
  }

  // Extract translation.
  const Vector3d position(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z);


  // Publish sensor message if an obstacle is within range.
  std::vector<Vector3d> obstacle_positions;
  std::vector<double> obstacle_radii;

  if (space_->SenseObstacles(position, sensor_radius_,
                            obstacle_positions, obstacle_radii)) {
    // Saw at least one obstacle, so convert to message and publish.
    meta_planner_msgs::SensorMeasurement msg;
    msg.num_obstacles = obstacle_positions.size();

    for (size_t ii = 0; ii < obstacle_positions.size(); ii++) {
      geometry_msgs::Vector3 p;
      p.x = obstacle_positions[ii](0);
      p.y = obstacle_positions[ii](1);
      p.z = obstacle_positions[ii](2);

      msg.positions.push_back(p);
      msg.radii.push_back(obstacle_radii[ii]);
    }

    sensor_pub_.publish(msg);
  }

  // Visualize the environment.
  space_->Visualize(environment_pub_, fixed_frame_id_);

   // Visualize the sensor radius.
  visualization_msgs::Marker sensor_radius_marker;
  sensor_radius_marker.ns = "sensor";
  sensor_radius_marker.header.frame_id = robot_frame_id_;
  sensor_radius_marker.header.stamp = right_now;
  sensor_radius_marker.id = 0;
  sensor_radius_marker.type = visualization_msgs::Marker::SPHERE;
  sensor_radius_marker.action = visualization_msgs::Marker::ADD;

  sensor_radius_marker.scale.x = 2.0 * sensor_radius_;
  sensor_radius_marker.scale.y = 2.0 * sensor_radius_;
  sensor_radius_marker.scale.z = 2.0 * sensor_radius_;

  sensor_radius_marker.color.a = 0.2;
  sensor_radius_marker.color.r = 0.8;
  sensor_radius_marker.color.g = 0.0;
  sensor_radius_marker.color.b = 0.2;

  sensor_radius_pub_.publish(sensor_radius_marker);
}

} //\namespace meta
