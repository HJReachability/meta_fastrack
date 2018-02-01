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
// Defines the Trajectory struct.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/trajectory.h>

namespace meta {

// Factory constructor from times, states, values.
Trajectory::Ptr Trajectory::
Create(const std::vector<double>& times,
       const std::vector<VectorXd>& states,
       const std::vector<ValueFunctionId>& control_values,
       const std::vector<ValueFunctionId>& bound_values,
       const std::vector<double>& collision_probs) {
  Trajectory::Ptr ptr(new Trajectory());

  // Number of entries in trajectory.
  size_t num_waypoints = states.size();

#ifdef ENABLE_DEBUG_MESSAGES
  if (states.size() != times.size() ||
      states.size() != control_values.size() ||
      states.size() != bound_values.size()) {
    ROS_WARN("Inconsistent number of states, times, and values.");
    num_waypoints = std::min(states.size(),
                             std::min(times.size(),
                                      std::min(control_values.size(),
                                               bound_values.size())));
  }
#endif

  for (size_t ii = 0; ii < num_waypoints; ii++) {
    const double collision = (collision_probs.size() == num_waypoints) ?
      collision_probs[ii] : 0.0;
    ptr->Add(times[ii], states[ii], control_values[ii], 
	     bound_values[ii], collision);
  }

  return ptr;
}

// Factory constructor from ROS message and list of ValueFunctions.
Trajectory::Ptr Trajectory::
Create(const meta_planner_msgs::Trajectory::ConstPtr& msg) {
  Trajectory::Ptr ptr(new Trajectory());

  // Number of entries in trajectory.
  size_t num_waypoints = msg->num_waypoints;

  for (size_t ii = 0; ii < num_waypoints; ii++) {
    // Convert state message to VectorXd.
    const VectorXd state = utils::Unpack(msg->states[ii]);

    // Add to this trajectory.
    const double collision = 
      (msg->collision_probs.size() == num_waypoints) ? 
      msg->collision_probs[ii] : 0.0;
    ptr->Add(msg->times[ii], 
	     state,
             msg->control_value_function_ids[ii],
             msg->bound_value_function_ids[ii],
	     collision);
  }

  return ptr;
}

// Factory constructor to create a Trajectory as the remainder of the
// given Trajectory after the specified time point.
Trajectory::Ptr Trajectory::
Create(const Trajectory::ConstPtr& other, double start) {
  Trajectory::Ptr traj = Trajectory::Create();

  // Insert the current state at the start time.
  traj->Add(start,
            other->GetState(start),
            other->GetControlValueFunction(start),
            other->GetBoundValueFunction(start),
	    other->GetCollisionProbability(start));

  // Insert the rest of the states in the other trajectory.
  // Get a const iterator to a time in the other trajectory >= start time.
  std::map<double, StateValue>::const_iterator iter =
    other->map_.lower_bound(start);

  // Iterate through all remaining states.
  while (iter != other->map_.end()) {
    traj->Add(iter->first,
              iter->second.state_,
              iter->second.control_value_,
              iter->second.bound_value_,
	      iter->second.collision_prob_);
    iter++;
  }

  return traj;
}

// Convert to ROS message.
meta_planner_msgs::Trajectory Trajectory::ToRosMessage() const {
  meta_planner_msgs::Trajectory traj_msg;
  traj_msg.num_waypoints = Size();

  // Iterate through the trajectory and append to message.
  for (const auto& pair : map_) {
    // Extract state.
    const meta_planner_msgs::State state_msg =
      utils::PackState(pair.second.state_);

    // Update message.
    traj_msg.states.push_back(state_msg);
    traj_msg.times.push_back(pair.first);
    traj_msg.control_value_function_ids.push_back(pair.second.control_value_);
    traj_msg.bound_value_function_ids.push_back(pair.second.bound_value_);
    traj_msg.collision_probs.push_back(pair.second.collision_prob_);
  }

  return traj_msg;
}

// Find the state corresponding to a particular time via linear interpolation.
VectorXd Trajectory::GetState(double time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to interpolate an empty trajectory.");
    throw std::underflow_error("Tried to interpolate an empty trajectory.");
  }
#endif

  // Get a const iterator to a time not less than this one.
  std::map<double, StateValue>::const_iterator iter = map_.lower_bound(time);

#ifdef ENABLE_DEBUG_MESSAGES
  if (iter == map_.end()) {
    ROS_WARN_THROTTLE(1.0, "Could not interpolate. Time was too late.");
    return LastState();
  }

  if (iter == map_.begin()) {
    ROS_WARN_THROTTLE(1.0, "Could not interpolate. Time was too early.");
    return FirstState();
  }
#endif

  // Get the state with time not later than this time.
  const double upper_time = iter->first;
  const VectorXd& upper_state = iter->second.state_;

  if (upper_time == time)
    return upper_state;

  // Get the state with time earlier than this time.
  iter--;
  const double lower_time = iter->first;
  const VectorXd& lower_state = iter->second.state_;

  // Linear interpolation.
  return lower_state + (upper_state - lower_state) *
    (time - lower_time) / (upper_time - lower_time);
}

// Return the ID of the value function being used at this time.
ValueFunctionId Trajectory::GetControlValueFunction(double time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to interpolate an empty trajectory.");
    throw std::underflow_error("Tried to interpolate an empty trajectory.");
  }
#endif

  // Get a const iterator to a time not less than this one.
  std::map<double, StateValue>::const_iterator iter = map_.lower_bound(time);

  // Catch end.
  if (iter == map_.end()) {
    ROS_WARN("This time occurred after the trajectory.");
    return (--iter)->second.control_value_;
  }

  // Catch equality.
  if (iter->first == time)
    return iter->second.control_value_;

  // Catch beginning. Note this occurs after equality check, so if this is
  // true then the specified time must occur before the start of the trajectory.
  if (iter == map_.begin()) {
    ROS_WARN("This time occurred before the trajectory.");
    return iter->second.control_value_;
  }

  // Regular case: iter is after the specified time.
  return (--iter)->second.control_value_;
}

// Get the collision probability at this time.
double Trajectory::GetCollisionProbability(double time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to interpolate an empty trajectory.");
    throw std::underflow_error("Tried to interpolate an empty trajectory.");
  }
#endif

  // Get a const iterator to a time not less than this one.
  std::map<double, StateValue>::const_iterator iter = map_.lower_bound(time);

  // Catch end.
  if (iter == map_.end()) {
    ROS_WARN("This time occurred after the trajectory.");
    return (--iter)->second.collision_prob_;
  }

  // Catch equality.
  if (iter->first == time)
    return iter->second.collision_prob_;

  // Catch beginning. Note this occurs after equality check, so if this is
  // true then the specified time must occur before the start of the trajectory.
  if (iter == map_.begin()) {
    ROS_WARN("This time occurred before the trajectory.");
    return iter->second.collision_prob_;
  }

  // Regular case: iter is after the specified time.
  return (--iter)->second.collision_prob_;
}

// Return the ID of the value function being used at this time.
ValueFunctionId Trajectory::GetBoundValueFunction(double time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to interpolate an empty trajectory.");
    throw std::underflow_error("Tried to interpolate an empty trajectory.");
  }
#endif

  // Get a const iterator to a time not less than this one.
  std::map<double, StateValue>::const_iterator iter = map_.lower_bound(time);

  // Catch end.
  if (iter == map_.end()) {
    ROS_WARN("This time occurred after the trajectory.");
    return (--iter)->second.bound_value_;
  }

  // Catch equality.
  if (iter->first == time)
    return iter->second.bound_value_;

  // Catch beginning. Note this occurs after equality check, so if this is
  // true then the specified time must occur before the start of the trajectory.
  if (iter == map_.begin()) {
    ROS_WARN("This time occurred before the trajectory.");
    return iter->second.bound_value_;
  }

  // Regular case: iter is after the specified time.
  return (--iter)->second.bound_value_;
}

// Swap out the control value function in this trajectory and update time
// stamps accordingly.
void Trajectory::ExecuteSwitch(ValueFunctionId value,
                               ros::ServiceClient& best_time_srv) {
  std::map<double, StateValue> switched;

  double last_time = FirstTime();
  VectorXd last_state = FirstState();

  // HACK! Assuming state layout.
  Vector3d last_position(last_state(0), last_state(1), last_state(2));
  for (auto iter = map_.begin(); iter != map_.end(); iter++) {
    // (1) Compute time for this state from last_time.
    const ValueFunctionId bound = iter->second.bound_value_;
    const VectorXd state = iter->second.state_;
    const double collision = iter->second.collision_prob_;

    // HACK! Still assuming state layout.
    const Vector3d position(state(0), state(1), state(2));

    double dt = 10.0;
    value_function::GeometricPlannerTime t;
    t.request.id = value;
    t.request.start = utils::Pack(last_position);
    t.request.stop = utils::Pack(position);
    if (!best_time_srv)
      ROS_WARN("Trajectory: Best time server disconnected. Assuming fixed dt.");
    else if (!best_time_srv.call(t))
      ROS_ERROR("Trajectory: Error calling best time server.");
    else
      dt = t.response.time;

    const double time = last_time + dt;

    // (2) Insert this tuple into 'switched'.
    switched.insert({ time, StateValue(state, value, bound, collision) });

    // (3) Update last_state, last_time, and last_position.
    last_state = state;
    last_position = position;
    last_time = time;
  }

  // Swap out map_ for switched.
  map_.clear();
  map_.insert(switched.begin(), switched.end());
}

// Adjust the time stamps for this trajectory to start at the given time.
void Trajectory::ResetStartTime(double start) {
  std::map<double, StateValue> reset;

  // Compute difference between this start time and current start time.
  const double delay = start - FirstTime();

  // Loop over all pairs in the map and update time in reset.
  for (const auto& pair : map_) {
    const double time = pair.first + delay;

    // Insert into reset.
    reset.insert({ time, pair.second });
  }

  // Swap out map_ for reset.
  map_.clear();
  map_.insert(reset.begin(), reset.end());
}

// Visualize this trajectory in RVIZ.
void Trajectory::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {
  if (pub.getNumSubscribers() <= 0)
    return;

  // Set up spheres marker.
  visualization_msgs::Marker spheres;
  spheres.ns = "spheres";
  spheres.header.frame_id = frame_id;
  spheres.header.stamp = ros::Time::now();
  spheres.id = 0;
  spheres.type = visualization_msgs::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::Marker::ADD;

  spheres.scale.x = 0.08;
  spheres.scale.y = 0.08;
  spheres.scale.z = 0.15;

#if 0
  spheres.color.a = 0.5;
  spheres.color.r = 0.7;
  spheres.color.g = 0.0;
  spheres.color.b = 0.8;
#endif

  // Set up line strip marker.
  visualization_msgs::Marker lines;
  lines.ns = "lines";
  lines.header.frame_id = frame_id;
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;

  lines.scale.x = 0.03;
#if 0
  lines.color.a = 0.5;
  lines.color.r = 0.0;
  lines.color.g = 0.0;
  lines.color.b = 0.6;
#endif

  size_t idx = 0;
  // Iterate through the trajectory and append to markers.
  for (const auto& pair : map_) {
    // Extract point. HACK! Assuming state layout.
    geometry_msgs::Point p;
    p.x = pair.second.state_(0);
    p.y = pair.second.state_(1);
    p.z = pair.second.state_(2);

    const std_msgs::ColorRGBA c = Colormap(pair.first);

    // Handle 'spheres' marker.
    spheres.points.push_back(p);
    spheres.colors.push_back(c);

    // Handle 'lines' marker.
    lines.points.push_back(p);
    lines.colors.push_back(c);


    // Set up text marker.
    visualization_msgs::Marker text;
    text.ns = "text";
    text.header.frame_id = frame_id;
    text.header.stamp = ros::Time::now();
    text.id = idx++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;

    text.scale.x = 0.1;
    text.scale.y = 0.1;
    text.scale.z = 0.1;

    text.pose.position.x = p.x;
    text.pose.position.y = p.y;
    text.pose.position.z = p.z;
    text.pose.orientation.x = 0.0;
    text.pose.orientation.y = 0.0;
    text.pose.orientation.z = 0.0;
    text.pose.orientation.w = 1.0;

    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.text = std::to_string(pair.second.collision_prob_);
    pub.publish(text);
  }

  // Publish markers. Only publish 'lines' if more than one point in trajectory.
  pub.publish(spheres);

  if (Size() > 1)
    pub.publish(lines);
}

// Compute the color (on a red-blue colormap) at a particular index.
std_msgs::ColorRGBA Trajectory::Colormap(double time) const {
  std_msgs::ColorRGBA color;

#ifdef ENABLE_DEBUG_MESSAGES
  if (time < FirstTime() || time > LastTime()) {
    ROS_ERROR("Tried to compute colormap for out-of-bounds index.");

    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
    return color;
  }
#endif

  // Compute total trajectory timescale.
  double total_time = Time();

  const double kSmallNumber = 1e-8;
  if (total_time < kSmallNumber) {
    ROS_WARN("Total time length of trajectory is too small.");
    total_time = kSmallNumber;
  }

  color.r = 0.0;//(time - FirstTime()) / total_time;
  color.g = 0.0;
  color.b = 1.0; //- color.r;
  color.a = 0.6;

  return color;
}

// Print this trajectory to stdout.
void Trajectory::Print(const std::string& prefix) const {
  std::cout << prefix << std::endl;
  for (const auto& pair : map_)
    std::cout << pair.first << " -- "
              << pair.second.state_.transpose() << std::endl;
}

} //\namespace meta
