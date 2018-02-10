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
// Defines a Box environment with spherical obstacles. For simplicity, this
// does not bother with a kdtree index to speed up collision queries, since
// it is only for a simulated demo.
//
///////////////////////////////////////////////////////////////////////////////

#include <demo/lanterns_in_box.h>

namespace meta {

// Factory method. Use this instead of the constructor.
LanternsInBox::Ptr LanternsInBox::Create() {
  LanternsInBox::Ptr ptr(new LanternsInBox());
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
LanternsInBox::LanternsInBox()
  : Box(),
    tf_listener_(tf_buffer_) {}

// Initialize this environment.
void LanternsInBox::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "lanterns");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;

  // Update lantern positions.
  UpdateLanternPositions();

  return true;
}

// Load all parameters.
bool LanternsInBox::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Switching bound server.
  if (!nl.getParam("srv/switching_bound", switching_bound_name_)) return false;

  // Timer dt.
  if (!nl.getParam("lantern_dt", timer_dt_)) return false;

  // Frames.
  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("frames/lanterns", lantern_frame_ids_)) return false;
  points_.resize(lantern_frame_ids_.size());

  // Radius of lanterns.
  if (!nl.getParam("lantern_radius", radius_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool LanternsInBox::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Server.
  ros::service::waitForService(switching_bound_name_.c_str());
  switching_bound_srv_ =
    nl.serviceClient<value_function_srvs::SwitchingTrackingBoundBox>(
      switching_bound_name_.c_str(), true);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(timer_dt_),
    &LanternsInBox::TimerCallback, this);

  return true;
}

// Query tf for lantern poses.
void LanternsInBox::UpdateLanternPositions() {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!initialized_) {
    ROS_WARN("%s: Tried to update lantern positions before initializing.",
             name_.c_str());
    return;
  }
#endif

  // Read position from TF.
  const ros::Time right_now = ros::Time::now();

  // Get the current transform from tf.
  geometry_msgs::TransformStamped tf;

  for (size_t ii = 0; ii < lantern_frame_ids_.size(); ii++) {
    try {
      tf = tf_buffer_.lookupTransform(
        fixed_frame_id_.c_str(), lantern_frame_ids_[ii].c_str(), ros::Time(0));
    } catch(tf2::TransformException &ex) {
      ROS_WARN("%s: %s", name_.c_str(), ex.what());
      ROS_WARN("%s: Could not determine current position of lantern %zu.",
               name_.c_str(), ii);
      continue;
    }

    // Extract translation.
    points_[ii] = Vector3d(tf.transform.translation.x,
                           tf.transform.translation.y,
                           tf.transform.translation.z);
  }
}

// Timer callback to update lantern positions.
void LanternsInBox::TimerCallback(const ros::TimerEvent& e) {
  UpdateLanternPositions();
}

// Inherited collision checker from Box needs to be overwritten.
// Takes in incoming and outgoing value functions. See planner.h for details.
bool LanternsInBox::IsValid(const Vector3d& position,
                            ValueFunctionId incoming_value,
                            ValueFunctionId outgoing_value) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check an uninitialized LanternsInBox.",
             name_.c_str());
    return false;
  }
#endif

  // Make sure server is up.
  if (!switching_bound_srv_) {
    ROS_WARN("%s: Switching bound server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    switching_bound_srv_ = nl.serviceClient<value_function_srvs::SwitchingTrackingBoundBox>(
      switching_bound_name_.c_str(), true);

    return false;
  }

  // No obstacles. Just check bounds.
  value_function_srvs::SwitchingTrackingBoundBox bound;
  bound.request.from_id = incoming_value;
  bound.request.to_id = outgoing_value;
  if (!switching_bound_srv_.call(bound)) {
    ROS_ERROR("%s: Error calling switching bound server.", name_.c_str());
    return false;
  }

  if (position(0) < lower_(0) + bound.response.x ||
      position(0) > upper_(0) - bound.response.x ||
      position(1) < lower_(1) + bound.response.y ||
      position(1) > upper_(1) - bound.response.y ||
      position(2) < lower_(2) + bound.response.z ||
      position(2) > upper_(2) - bound.response.z)
    return false;

  // Check against each obstacle.
  const Vector3d bound_vector(
    bound.response.x, bound.response.y, bound.response.z);

  for (size_t ii = 0; ii < points_.size(); ii++) {
    const Vector3d& p = points_[ii];

    // Compute signed distance between position and obstacle center.
    const Vector3d signed_distance = p - position;

    // Find closest point in the tracking bound to the obstacle center.
    Vector3d closest_point;
    for (size_t jj = 0; jj < 3; jj++) {
      if (signed_distance(jj) >= 0.0) {
        if (signed_distance(jj) >= bound_vector(jj))
          closest_point(jj) = position(jj) + bound_vector(jj);
        else
          closest_point(jj) = p(jj);
      } else {
        if (signed_distance(jj) <= -bound_vector(jj))
          closest_point(jj) = position(jj) - bound_vector(jj);
        else
          closest_point(jj) = p(jj);
      }
    }

    // Check distance to closest point.
    if ((closest_point - p).norm() <= radius_)
      return false;
  }

  return true;
}


// Checks for obstacles within a sensing radius. Returns true if at least
// one obstacle was found.
bool LanternsInBox::SenseObstacles(const Vector3d& position, double sensor_radius,
                                std::vector<Vector3d>& obstacle_positions,
                                std::vector<double>& obstacle_radii) const {
  obstacle_positions.clear();
  obstacle_radii.clear();

  for (size_t ii = 0; ii < points_.size(); ii++){
    if ((position - points_[ii]).norm() <= radius_ + sensor_radius) {
      obstacle_positions.push_back(points_[ii]);
      obstacle_radii.push_back(radius_);
    }
  }

  return obstacle_positions.size() > 0;
}

// Checks if a given obstacle is in the environment.
bool LanternsInBox::IsObstacle(const Vector3d& obstacle_position,
                            double obstacle_radius) const {
  for (size_t ii = 0; ii < points_.size(); ii++)
    if ((obstacle_position - points_[ii]).norm() < 1e-8 &&
        std::abs(obstacle_radius - radius_) < 1e-8)
      return true;

  return false;
}


// Inherited visualizer from Box needs to be overwritten.
void LanternsInBox::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {
  if (pub.getNumSubscribers() <= 0)
    return;

  // Set up box marker.
  visualization_msgs::Marker cube;
  cube.ns = "cube";
  cube.header.frame_id = frame_id;
  cube.header.stamp = ros::Time::now();
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.color.a = 0.5;
  cube.color.r = 0.3;
  cube.color.g = 0.7;
  cube.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  cube.scale.x = upper_(0) - lower_(0);
  center.x = lower_(0) + 0.5 * cube.scale.x;

  cube.scale.y = upper_(1) - lower_(1);
  center.y = lower_(1) + 0.5 * cube.scale.y;

  cube.scale.z = upper_(2) - lower_(2);
  center.z = lower_(2) + 0.5 * cube.scale.z;

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish cube marker.
  pub.publish(cube);

  // Visualize obstacles as spheres.
  for (size_t ii = 0; ii < points_.size(); ii++){
    visualization_msgs::Marker sphere;
    sphere.ns = "sphere";
    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.id = static_cast<int>(ii);
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.scale.x = 2.0 * radius_;
    sphere.scale.y = 2.0 * radius_;
    sphere.scale.z = 2.0 * radius_;

    sphere.color.a = 0.9;
    sphere.color.r = 0.7;
    sphere.color.g = 0.5;
    sphere.color.b = 0.5;

    geometry_msgs::Point p;
    const Vector3d point = points_[ii];
    p.x = point(0);
    p.y = point(1);
    p.z = point(2);

    sphere.pose.position = p;

    // Publish sphere marker.
    pub.publish(sphere);
  }

}

} //\namespace meta
