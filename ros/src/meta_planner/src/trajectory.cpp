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
// Templated class to hold timestamped sequences of states and rapidly
// interpolate between states linearly.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/trajectory/trajectory.h>

#include <meta_planner_msgs/PlannerState.h>
#include <meta_planner_msgs/Trajectory.h>
//#include <fastrack_msgs/Trajectory.h>
#include <fastrack/utils/types.h>
#include <fastrack_msgs/State.h>

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace meta_planner {
namespace trajectory {

Trajectory::Trajectory(const std::list<Trajectory>& trajs) {
  for (const auto& traj : trajs) {
    // Concatenate states and times to existing lists.
    previous_planner_states_.insert(previous_planner_states_.end(),
                                    traj.previous_planner_states_.begin(),
                                    traj.previous_planner_states_.end());
    next_planner_states_.insert(next_planner_states_.end(),
                                traj.next_planner_states_.begin(),
                                traj.next_planner_states_.end());
    positions_.insert(positions_.end(), traj.positions_.begin(),
                      traj.positions_.end());
    previous_planner_id_.insert(previous_planner_id_.end(),
                                traj.previous_planner_id_.begin(),
                                traj.previous_planner_id_.end());
    next_planner_id_.insert(next_planner_id_.end(),
                            traj.next_planner_id_.begin(),
                            traj.next_planner_id_.end());

    // Reset first time to match last time of previous trajectory.
    const double time_offset =
        (times_.empty()) ? 0.0 : times_.back() - traj.times_.front();

    for (size_t ii = 0; ii < traj.times_.size(); ii++)
      times_.push_back(traj.times_[ii] + time_offset);
  }

  // Check to make sure time is monotonic increasing.
  for (size_t ii = 1; ii < times_.size(); ii++) {
    if (times_[ii - 1] > times_[ii])
      ROS_WARN("Trajectory: time was not monotone.");
  }
}

Trajectory::Trajectory(
    const std::vector<fastrack_msgs::State>& previous_planner_states,
    const std::vector<fastrack_msgs::State>& next_planner_states,
    const std::vector<geometry_msgs::Vector3>& positions,
    const std::vector<double>& times,
    const std::vector<size_t>& previous_planner_id,
    const std::vector<size_t>& next_planner_id)
    : previous_planner_states_(previous_planner_states),
      next_planner_states_(next_planner_states),
      positions_(positions),
      times_(times),
      previous_planner_id_(previous_planner_id),
      next_planner_id_(next_planner_id) {
  // Warn if state/time lists are not the same length and truncate
  // the longer one to match the smaller.
  if (previous_planner_states_.size() != times_.size()) {
    ROS_ERROR("Trajectory: states/times are not the same length.");

    // Resize the shorter one.
    size_t num_elements =
        std::min(previous_planner_states_.size(), times_.size());
    num_elements = std::min(num_elements, next_planner_states_.size());
    num_elements = std::min(num_elements, positions_.size());
    num_elements = std::min(num_elements, previous_planner_id_.size());
    num_elements = std::min(num_elements, next_planner_id_.size());

    previous_planner_states_.resize(num_elements);
    next_planner_states_.resize(num_elements);
    positions_.resize(num_elements);
    times_.resize(num_elements);
    previous_planner_id_.resize(num_elements);
    next_planner_id_.resize(num_elements);
  }

  // Make sure times are sorted. Overwrite any inversions with the larger
  // time as we move left to right in the list.
  for (size_t ii = 1; ii < times_.size(); ii++) {
    if (times_[ii - 1] > times_[ii]) {
      ROS_ERROR("Trajectory: fixing an inversion in the list of times.");
      times_[ii] = times_[ii + 1];
    }
  }
}

Trajectory::Trajectory(const meta_planner_msgs::Trajectory::ConstPtr& msg) {
  size_t num_elements = msg->states.size();

  // Get size carefully.
  if (msg->states.size() != msg->times.size()) {
    ROS_ERROR("Trajectory: states/times are not the same length.");
    num_elements = std::min(msg->states.size(), msg->times.size());
  }

  // Unpack message.
  for (size_t ii = 0; ii < num_elements; ii++) {
    previous_planner_states_.push_back(msg->states[ii].previous_planner_state);
    next_planner_states_.push_back(msg->states[ii].next_planner_state);
    positions_.push_back(msg->states[ii].position);
    previous_planner_id_.push_back(msg->states[ii].previous_planner_id);
    next_planner_id_.push_back(msg->states[ii].next_planner_id);
    times_.push_back(msg->times[ii]);
  }
}

fastrack_msgs::State Trajectory::Interpolate(
    double t, geometry_msgs::Vector3* position) const {
  // Get an iterator pointing to the first element in times_ that does
  // not compare less than t.
  const auto iter = std::lower_bound(times_.begin(), times_.end(), t);

  // Catch case where iter points to the beginning of the list.
  // This will happen if t occurs before the first time in the list.
  if (iter == times_.begin()) {
    ROS_WARN_THROTTLE(1.0, "Trajectory: interpolating before first time.");

    if (position) *position = positions_.front();
    return previous_planner_states_.front();
  }

  // Catch case where iter points to the end of the list.
  // This will happen if t occurs after the last time in the list.
  if (iter == times_.end()) {
    ROS_WARN_THROTTLE(1.0, "Trajectory: interpolating after the last time.");

    if (position) *position = positions_.back();
    return next_planner_states_.back();
  }

  // Iterator definitely points to somewhere in the middle of the list.
  // Get indices sandwiching t.
  const size_t hi = (iter - times_.begin());
  const size_t lo = hi - 1;

  // Linearly interpolate states.
  const double frac = (t - times_[lo]) / (times_[hi] - times_[lo]);

  fastrack_msgs::State interpolated;
  const size_t planner_state_dimension = next_planner_states_[lo].x.size();
  for (size_t ii = 0; ii < planner_state_dimension; ii++)
    interpolated.x.push_back((1.0 - frac) * next_planner_states_[lo].x[ii] +
                             frac * previous_planner_states_[hi].x[ii]);

  if (position) {
    position->x = (1.0 - frac) * positions_[lo].x + frac * positions_[hi].x;
    position->y = (1.0 - frac) * positions_[lo].y + frac * positions_[hi].y;
    position->z = (1.0 - frac) * positions_[lo].z + frac * positions_[hi].z;
  }

  return interpolated;
}

void Trajectory::ResetFirstTime(double t) {
  if (times_.size() == 0) {
    ROS_ERROR("Trajectory: Tried to reset first time of empty trajectory.");
    return;
  }

  const double constant_shift = t - times_.front();
  for (size_t ii = 0; ii < times_.size(); ii++) times_[ii] += constant_shift;
}

meta_planner_msgs::Trajectory Trajectory::ToRos() const {
  meta_planner_msgs::Trajectory msg;

  for (size_t ii = 0; ii < Size(); ii++) {
    meta_planner_msgs::PlannerState planner_state;
    planner_state.previous_planner_state = previous_planner_states_[ii];
    planner_state.next_planner_state = next_planner_states_[ii];
    planner_state.position = positions_[ii];
    planner_state.previous_planner_id = previous_planner_id_[ii];
    planner_state.next_planner_id = next_planner_id_[ii];

    msg.states.push_back(planner_state);
    msg.times.push_back(times_[ii]);
  }

  return msg;
}

void Trajectory::Visualize(const ros::Publisher& pub,
                           const std::string& frame) const {
  if (pub.getNumSubscribers() == 0) {
    ROS_WARN_THROTTLE(1.0, "Trajectory: I'm lonely. Please subscribe.");
    return;
  }

  // Set up spheres marker.
  visualization_msgs::Marker spheres;
  spheres.ns = "spheres";
  spheres.header.frame_id = frame;
  spheres.header.stamp = ros::Time::now();
  spheres.id = 0;
  spheres.type = visualization_msgs::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::Marker::ADD;
  spheres.scale.x = 0.1;
  spheres.scale.y = 0.1;
  spheres.scale.z = 0.1;

  // Set up line strip marker.
  visualization_msgs::Marker lines;
  lines.ns = "lines";
  lines.header.frame_id = frame;
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.05;

  // Populate markers.
  for (size_t ii = 0; ii < Size(); ii++) {
    geometry_msgs::Point p;
    p.x = positions_[ii].x;
    p.y = positions_[ii].y;
    p.z = positions_[ii].z;
    const std_msgs::ColorRGBA c = Colormap(times_[ii]);

    spheres.points.push_back(p);
    spheres.colors.push_back(c);
    lines.points.push_back(p);
    lines.colors.push_back(c);
  }

  // Publish.
  pub.publish(spheres);
  if (Size() > 1) pub.publish(lines);
}

std_msgs::ColorRGBA Trajectory::Colormap(double t) const {
  std_msgs::ColorRGBA c;

  c.r = (Size() == 0 || Duration() < 1e-8)
            ? 0.0
            : std::max(0.0, std::min(1.0, (t - times_.front()) / Duration()));
  c.g = 0.0;
  c.b = 1.0 - c.r;
  c.a = 0.9;

  return c;
}

}  //\namespace trajectory
}  //\namespace meta_planner
