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
// Defines the WaypointTree class. The WaypointTree class handles queries like
// finding the nearest k points, as well as the length (in time) of the
// shortest path to the goal.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/waypoint_tree.h>

namespace meta {

WaypointTree::WaypointTree(const Vector3d& start,
                           ValueFunctionId start_value,
                           double start_time)
  : root_(Waypoint::Create(start, start_value, nullptr, nullptr)),
    start_time_(start_time) {
  kdtree_.Insert(root_);
}

// Add Waypoint to tree.
void WaypointTree::Insert(const Waypoint::ConstPtr& waypoint, bool is_terminal) {
  kdtree_.Insert(waypoint);

  if (is_terminal) {
    if (terminus_ == nullptr) {
      ROS_WARN("Set initial terminus.");
      terminus_ = waypoint;
    }
    else if (waypoint->traj_->LastTime() < terminus_->traj_->LastTime()) {
      ROS_WARN("Updated terminus.");
      terminus_ = waypoint;
    }
  }
}

// Get best total time (seconds) of any valid trajectory. Returns negative
// if no valid trajectory exists.
double WaypointTree::BestTime() const {
  if (terminus_ == nullptr)
    return std::numeric_limits<double>::infinity();

  return terminus_->traj_->LastTime() - start_time_;
}

// Get best (fastest) trajectory (if it exists).
Trajectory::Ptr WaypointTree::BestTrajectory() const {
  if (terminus_ == nullptr) {
    ROS_WARN("Tree did not reach to the terminus.");
    return nullptr;
  }

  Trajectory::Ptr traj = Trajectory::Create();

  // Walk back from the terminus, and append trajectories as we go.
  Waypoint::ConstPtr waypoint = terminus_;
  while (waypoint != nullptr && waypoint->traj_ != nullptr) {
    traj->Add(waypoint->traj_);
    waypoint = waypoint->parent_;
  }

  return traj;
}

} //\namespace meta
