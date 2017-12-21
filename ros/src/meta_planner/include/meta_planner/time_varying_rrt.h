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

#ifndef META_PLANNER_TIME_VARYING_RRT_H
#define META_PLANNER_TIME_VARYING_RRT_H

#include <meta_planner/planner.h>
#include <meta_planner/flann_tree.h>

#include <ros/ros.h>
#include <memory>

namespace meta {

class TimeVaryingRrt : public Planner {
public:
  typedef std::shared_ptr<TimeVaryingRrt> Ptr;
  typedef std::shared_ptr<const TimeVaryingRrt> ConstPtr;

  virtual ~TimeVaryingRrt() {}

  static TimeVaryingRrt::Ptr Create(ValueFunctionId incoming_value,
                                    ValueFunctionId outgoing_value,
                                    const Box::ConstPtr& space,
                                    const Dynamics::ConstPtr& dynamics,
                                    double max_connection_radius = 10.0,
                                    double collision_check_resolution = 0.1);

  // Derived classes must plan trajectories between two points.
  // Budget is the time the planner is allowed to take during planning.
  Trajectory::Ptr Plan(const Vector3d& start,
                       const Vector3d& stop,
                       double start_time = 0.0,
                       double budget = 0.1) const;

private:
  explicit TimeVaryingRrt(ValueFunctionId incoming_value,
                          ValueFunctionId outgoing_value,
                          const Box::ConstPtr& space,
                          const Dynamics::ConstPtr& dynamics,
                          double max_connection_radius,
                          double collision_check_resolution)
    : Planner(incoming_value, outgoing_value, space, dynamics),
      max_connection_radius_(max_connection_radius),
      collision_check_resolution_(collision_check_resolution) {}

  // Custom node struct.
  struct Node {
  public:
    typedef std::shared_ptr<const Node> ConstPtr;

    const Vector3d point_;
    const ConstPtr parent_;
    const double time_;

    // Factory method.
    static inline ConstPtr Create(const Vector3d& point, const ConstPtr& parent,
                                  double time) {
      ConstPtr ptr(new Node(point, parent, time));
      return ptr;
    }

  private:
    explicit Node(const Vector3d& point, const ConstPtr& parent, double time)
      : point_(point), parent_(parent), time_(time) {}
  }; //\struct Node

  // Collision check a line segment between the two points with the given
  // start and stop times. Returns true if the path is collision free and
  // false otherwise.
  bool CollisionCheck(const Vector3d& start, const Vector3d& stop,
                      double start_time, double stop_time) const;

  // Walk backward from the given node to the root to create a Trajectory.
  Trajectory::Ptr GenerateTrajectory(const Node::ConstPtr& node) const;

  // Maximum distance of any connection in the RRT.
  const double max_connection_radius_;

  // Maximum distance between test points on line segments being
  // collision checked.
  const double collision_check_resolution_;

}; //\ class TimeVaryingRrt

} //\namespace meta

#endif
