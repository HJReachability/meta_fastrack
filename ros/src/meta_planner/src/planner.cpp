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
// Defines the Planner abstract class interface. For now, all Planners must
// operate within a Box. This is because of the way in which subspaces are
// specified in the constructor.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/planner.h>

namespace meta {

// Initialize this class from a ROS node.
bool Planner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "planner");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load all parameters.
bool Planner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Sensor radius.
  if (!nl.getParam("srv/best_time", best_time_name_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool Planner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Server.
  best_time_srv_ = nl.serviceClient<value_function_srvs::GeometricPlannerTime>(
    best_time_name_.c_str(), true);

  return true;
}

// Shortest possible time to go from start to stop for this planner.
double Planner::
BestPossibleTime(const Vector3d& start, const Vector3d& stop) const {
  double best_time = std::numeric_limits<double>::infinity();

  // Make sure the server is up.
  if (!best_time_srv_) {
    ROS_WARN("%s: Best time server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    best_time_srv_ = nl.serviceClient<value_function_srvs::GeometricPlannerTime>(
      best_time_name_.c_str(), true);
    return best_time;
  }

  // Call the server.
  value_function_srvs::GeometricPlannerTime t;
  t.request.id = incoming_value_;
  t.request.start = utils::Pack(start);
  t.request.stop = utils::Pack(stop);
  if (!best_time_srv_.call(t))
    ROS_ERROR("%s: Error calling best time server.", name_.c_str());
  else
    best_time = t.response.time;

  return best_time;
}

} //\namespace meta
