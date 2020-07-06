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

#ifndef META_PLANNER_TRAJECTORY_TRAJECTORY_H
#define META_PLANNER_TRAJECTORY_TRAJECTORY_H

#include <meta_planner_msgs/PlannerState.h>
#include <meta_planner_msgs/Trajectory.h>
//#include <fastrack_msgs/Trajectory.h>
#include <fastrack/utils/types.h>
#include <fastrack_msgs/State.h>
#include <meta_planner_msgs/PlannerState.h>

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace meta_planner {
namespace trajectory {

class Trajectory {
 public:
  ~Trajectory() {}
  Trajectory() {}

  // Construct from a list of trajectories.
  explicit Trajectory(const std::list<Trajectory>& trajs);

  // Construct from lists of states and times.
  Trajectory(const std::vector<fastrack_msgs::State>& previous_planner_states,
             const std::vector<fastrack_msgs::State>& next_planner_states,
             const std::vector<geometry_msgs::Vector3>& positions,
             const std::vector<double>& times,
             const std::vector<size_t>& previous_planner_id,
             const std::vector<size_t>& next_planner_id);

  // Construct from a ROS message.
  explicit Trajectory(const meta_planner_msgs::Trajectory& msg);

  // Size (number of states in this Trajectory).
  size_t Size() const { return previous_planner_states_.size(); }
  bool Empty() const { return Size() == 0; }

  // Duration in seconds.
  double Duration() const { return times_.back() - times_.front(); }

  // First and last states/times.
  fastrack_msgs::State FirstState() const {
    return previous_planner_states_.front();
  }
  fastrack_msgs::State LastState() const {
    return previous_planner_states_.back();
  }
  double FirstTime() const { return times_.front(); }
  double LastTime() const { return times_.back(); }

  // First and last planner id.
  size_t FirstPlannerId() const { return next_planner_id_.front(); }
  size_t LastPlannerId() const { return previous_planner_id_.back(); }

  // Const accessors.
  const std::vector<fastrack_msgs::State>& PreviousPlannerStates() const {
    return previous_planner_states_;
  }
  const std::vector<fastrack_msgs::State>& NextPlannerStates() const {
    return next_planner_states_;
  }
  const std::vector<geometry_msgs::Vector3>& Positions() const {
    return positions_;
  }
  const std::vector<double>& Times() const { return times_; }

  // Interpolate at a particular time.
  meta_planner_msgs::PlannerState Interpolate(double t) const;

  // Reset first time and update all other times to preserve the adeltas.
  void ResetFirstTime(double t);

  // Convert to a ROS message.
  meta_planner_msgs::Trajectory ToRos() const;

  // Visualize this trajectory.
  void Visualize(const ros::Publisher& pub, const std::string& frame) const;

 private:
  // Custom colormap for the given time.
  std_msgs::ColorRGBA Colormap(double t) const;

  // Lists of states, positions, and times.
  std::vector<fastrack_msgs::State> previous_planner_states_;
  std::vector<fastrack_msgs::State> next_planner_states_;
  std::vector<geometry_msgs::Vector3> positions_;
  std::vector<size_t> previous_planner_id_;
  std::vector<size_t> next_planner_id_;
  std::vector<double> times_;
};  //\class Trajectory

}  //\namespace trajectory
}  //\namespace meta_planner

#endif
