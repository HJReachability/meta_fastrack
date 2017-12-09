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
// Classical RRT in 3D, but where collision checks are time-dependent.
// Inherits from the Planner base class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/time_varying_rrt.h>

#include <algorithm>

namespace meta {

// Factory method.
TimeVaryingRrt::Ptr TimeVaryingRrt::
Create(ValueFunctionId incoming_value, ValueFunctionId outgoing_value,
       const Box::ConstPtr& space, const Dynamics::ConstPtr& dynamics) {
  TimeVaryingRrt::Ptr ptr(new TimeVaryingRrt(
    incoming_value, outgoing_value, space, dynamics));
  return ptr;
}

// Derived classes must plan trajectories between two points.
// Budget is the time the planner is allowed to take during planning.
Trajectory::Ptr TimeVaryingRrt::
Plan(const Vector3d& start, const Vector3d& stop,
     double start_time, double budget) const {
  // Check that both start and stop are in bounds.
  if (!space_->IsValid(start, incoming_value_, outgoing_value_)) {
    ROS_WARN_THROTTLE(1.0, "TimeVaryingRrt: Start point was in collision or out of bounds.");
    return nullptr;
  }

  if (!space_->IsValid(stop, incoming_value_, outgoing_value_)) {
    ROS_WARN_THROTTLE(1.0, "TimeVaryingRrt: Stop point was in collision or out of bounds.");
    return nullptr;
  }

  // Root the RRT at the start point.
  const Node::ConstPtr root = Node::Create(start, nullptr, start_time);
  kdtree_.Insert(root);

  // Loop until our time budget has expired.
  const ros::Time begin = ros::Time::now();
  Node::ConstPtr terminus = nullptr;

  while ((ros::Time::now() - begin).toSec() < budget) {
    // Sample a new point.
    const Vector3d sample = space_.Sample();

    // Throw out this sample if it could never lead to improvement.
    if (terminus != nullptr && terminus->time_ - start_time <
        BestPossibleTime(start, sample) + BestPossibleTime(sample, stop))
      continue;

    // Find the nearest neighbor in our existing kdtree.
    const size_t kNumNeigbhbors = 1;
    const std::vector<Node::ConstPtr> neighbors =
      kdtree_.KnnSearch(sample, kNumNeigbhbors);

#ifdef ENABLE_DEBUG_MESSAGES
    if (neighbors.size() != kNumNeigbhbors) {
      // Should never get here.
      ROS_ERROR("TimeVaryingRrt: KnnSearch found the wrong number of neighbors.");
      return nullptr;
    }
#endif

    // Throw out this sample if it is too far from the nearest neighbor.
    if ((neighbors[0]->point_ - sample).norm() > max_connection_radius_)
      continue;

    // Compute the time at which we would get to the sample point.
    const double sample_time =
      neighbors[0]->time_ + BestPossibleTime(neighbors[0]->point_, sample);

    // Try to connect the sample to the nearest neighbor.
    if (!CollisionCheck(neighbors[0]->point_, sample,
                        neighbors[0]->time_, sample_time))
      continue;

    // Insert this point into the kdtree.
    const Node::ConstPtr sample_node =
      Node::Create(sample, neigbors[0], sample_time);
    kdtree_.Insert(sample_node);

    // Don't try to connect to the goal if it's too far away.
    if ((stop - sample).norm() > max_connection_radius_)
      continue;

    // Compute the time we would reach the goal.
    const double stop_time = sample_time + BestPossibleTime(sample, stop);

    // Only attempt to connect to the goal if it could lead to improvement.
    if (terminus != nullptr && terminus->time_ < stop_time)
      continue;

    // Try to connect the sample to the goal.
    if (!CollisionCheck(sample, stop, sample_time, stop_time))
      continue;

    // We've found a better path than we had before, so update the terminus.
    terminus = Node::Create(stop, sample, stop_time);
  }

  return GenerateTrajectory(terminus);
}

// Collision check a line segment between the two points with the given
// initial start time. Returns true if the path is collision free and
// false otherwise.
bool TimeVaryingRrt::CollisionCheck(const Vector3d& start, const Vector3d& stop,
                                    double start_time, double stop_time) const {
  // Compute the unit vector pointing from start to stop.
  const Vector3d direction = (stop - start) / (stop - start).norm();

  // Compute the dt between query points.
  const double dt =
    (stop_time - start_time) * collision_check_resolution_ / (stop - start).norm();

  // Start at the start point and walk until we get past the stop point.
  for (Vector3d query(start), double time = start_time; time < stop_time;
       query += collision_check_resolution_ * direction, time += dt) {
    if (!space_->IsValid(query, incoming_value_, outgoing_value_, time))
      return false;
  }

  return true;
}

// Walk backward from the given node to the root to create a Trajectory.
Trajectory::Ptr TimeVaryingRrt::GenerateTrajectory(
  const Node::ConstPtr& node) const {
  // Start with an empty list of positions and times.
  std::vector<Vector3d> positions;
  std::vector<double> times;

  // Populate these lists by walking backward, then reverse.
  for (Node::ConstPtr n = node; n != nullptr; n = n->parent_) {
    positions.push_back(n->point_);
    times.push_back(n->time_);
  }

  std::reverse(positions.begin(), positions.end());
  std::reverse(times.begin(), times.end());

  // Lift positions into states.
  const std::vector<VectorXd> states =
    dynamics_->LiftGeometricTrajectory(positions, times);

  // Create dummy list containing value function IDs.
  const std::vector<ValueFunctionId> values(states.size(), incoming_value_);

  // Create a trajectory.
  return Trajectory::Create(times, states, values, values);
}

} //\namespace meta
