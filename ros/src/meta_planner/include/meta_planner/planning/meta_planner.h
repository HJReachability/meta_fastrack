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
 *          Sylvia Herbert ( sylvia.lee.herbert@gmail.com )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the MetaPlanner class. The MetaPlanner samples random points in
// the state space and then spawns off different Planners to plan Trajectories
// between these points (RRT-style).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_META_PLANNER_H
#define META_PLANNER_META_PLANNER_H

#include <fastrack/state/position_velocity.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/planning/waypoint.h>
#include <meta_planner/planning/waypoint_tree.h>

#include <fastrack_msgs/State.h>
#include <meta_planner_msgs/ReplanRequest.h>
#include <meta_planner_msgs/SensorMeasurement.h>
#include <meta_planner_msgs/Trajectory.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>

namespace meta_planner {
namespace planning {

using fastrack::state::PositionVelocity;

template <typename S>
class MetaPlanner : private fastrack::Uncopyable {
 public:
  ~MetaPlanner() {}
  MetaPlanner() : initialized_(false) {}

  // Initialize this class from a ROS node.
  bool Initialize(const ros::NodeHandle& n);

 private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback to handle requests for new trajectory.
  void RequestTrajectoryCallback(
      const meta_planner_msgs::ReplanRequest::ConstPtr& msg);

  // Plan a trajectory from the given start to stop points, beginning at the
  // specified start time. Auto-publishes the result and returns whether
  // meta planning was successful.
  bool Plan(const fastrack_msgs::State& start, const fastrack_msgs::State& goal,
            double start_time, size_t initial_planner_id);

  // Convert previous/next planner IDs to row-major index.
  size_t ToFlatIndex(size_t previous_planner_id, size_t next_planner_id) const {
    return num_planners_ * previous_planner_id + next_planner_id;
  }

  // Convert from a tracker state (S) to a planner state fastrack msg.
  fastrack_msgs::State ToPlannerStateMsg(const S& x_in_tracker_space,
                                         size_t planner_id) const;

  // Convert planner state msg to position in 3D.
  Vector3d ToPosition(const fastrack_msgs::State& msg, size_t planner_id) const;

  // Convert between two planner state types.
  fastrack_msgs::State MetaPlanner<S>::ConvertPlannerStateMsgs(
      const fastrack_msgs::State& planner1_x, size_t planner1_id,
      size_t planner2_id) const;

  // List of planner services.
  // NOTE: this is actually a flattened matrix of planners, since
  // we're going to have separate planners depending on whether they're using
  // TEBs or SSBs in order to keep the fastrack interface intact.
  size_t num_planners_;
  std::vector<ros::ServiceClient> planner_srvs_;
  std::vector<std::string> planner_srv_names_;

  // List of planner state type names.
  std::vector<std::string> planner_state_types_;

  // Max time to spend searching for an optimal path.
  double max_runtime_;

  // Maximum distance between waypoints.
  double max_connection_radius_;

  // Publishers/subscribers and related topics.
  ros::Publisher traj_pub_;
  ros::Subscriber request_traj_sub_;

  std::string traj_topic_;
  std::string request_traj_topic_;

  // Frames.
  std::string fixed_frame_id_;

  // Are we in flight?
  bool in_flight_;

  // Initialization and naming.
  bool initialized_;
  std::string name_;
};

// ---------------------------- IMPLEMENTATION --------------------------- //

template <typename S>
fastrack_msgs::State MetaPlanner<S>::ToPlannerStateMsg(
    const S& x_in_tracker_space, size_t planner_id) const {
  fastrack_msgs::State msg;

  switch (planner_state_types_[ToFlatIndex(planner_id, planner_id)]) {
    case "PositionVelocity":
      PositionVelocity x_in_planner_space(x_in_tracker_space);
      msg = x_in_planner_space.ToRos();
      break;
    // TODO: can add more state conversions later if needed.
    default:
      ROS_FATAL("%s: You dummy. This is not a valid state type.",
                name_.c_str());
  }

  return msg;
}

template <typename S>
Vector3d MetaPlanner<S>::ToPosition(const fastrack_msgs::State& msg,
                                    size_t planner_id) const {
  Vector3d position;

  switch (planner_state_types_[ToFlatIndex(planner_id, planner_id)]) {
    case "PositionVelocity":
      PositionVelocity planner_x(msg);
      position = planner_x.Position();
      break;
    // TODO: can add more state conversions later if needed.
    default:
      ROS_FATAL("%s: You dummy. This is not a valid state type.",
                name_.c_str());
  }

  return position;
}

template <typename S>
fastrack_msgs::State MetaPlanner<S>::ConvertPlannerStateMsgs(
    const fastrack_msgs::State& planner1_x, size_t planner1_id,
    size_t planner2_id) const {
  fastrack_msgs::State msg;

  const size_t flattened_id1 = ToFlatIndex(planner1_id, planner1_id);
  const size_t flattened_id2 = ToFlatIndex(planner2_id, planner2_id);

  const auto& type1 = planner_state_types_[flattened_id1];
  const auto& type2 = planner_state_types_[flattened_id2];

  if (type1 == "PositionVelocity" && type2 == "PositionVelocity")
    msg = planner1_x;
  // TODO: can add more state conversions later if needed.
  else
    ROS_FATAL("%s: You dummy. This is not a valid state type.", name_.c_str());

  return msg;
}

template <typename S>
bool MetaPlanner<S>::Plan(const fastrack_msgs::State& start,
                          const fastrack_msgs::State& goal, double start_time,
                          size_t initial_planner_id) {
  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time current_time = ros::Time::now();
  WaypointTree tree(S(start).Position(), start, initial_planner_id, start_time);

  bool found = false;
  while ((ros::Time::now() - current_time).toSec() < max_runtime_) {
    // (2) Sample a new point in the state space.
    const S sample = S::Sample();

    // (3) Find the nearest neighbor in position space.
    constexpr size_t kNumNeighbors = 1;
    const std::vector<Waypoint::ConstPtr> neighbors =
        tree.KnnSearch(sample.Position(), kNumNeighbors);

    // Throw out this sample if too far from the nearest point.
    if (neighbors.size() != kNumNeighbors ||
        (neighbors[0]->point_ - sample.Position()).norm() >
            max_connection_radius_)
      continue;

    Waypoint::ConstPtr neighbor = neighbors[0];

    // Is the neighbor the root.
    const bool is_root = neighbor->traj_.Empty();

    // (4) Plan a trajectory (starting with the first planner and
    // ending with the last planner).
    Trajectory traj;
    size_t planner_used;
    for (size_t ii = 0; ii < num_planners_; ii++) {
      // Convert this to a fastrack_msgs::State in the planner's state space.
      const fastrack_msgs::State sample_planner_x =
          ToPlannerStateMsg(sample, ii);

      // Also convert the neighbor's state to this (ii'th) planner state.
      const fastrack_msgs::State neighbor_planner_x =
          ConvertPlannerStateMsgs(neighbor->state_, neighbor->planner_id_, ii);

      // Plan using 10% of the available total runtime.
      // HACK! This is just a heuristic and could easily be changed.
      const double time = (is_root) ? start_time : neighbor->traj.LastTime();

      // Call planner ii's service.
      fastrack_srvs::Replan srv;
      srv.req.req.start = neighbor_planner_x;
      srv.req.req.goal = sample_planner_x;
      srv.req.req.start_time = time;

      if (!planner_srvs_[ToFlatIndex(neighbor->planner_id, ii)]) {
        ROS_ERROR("%s: Server failed for planner %zu=>%zu.", name_.c_str(),
                  neighbor->planner_id, ii);
        continue;
      }

      // Convert service response (fastrack_msgs::Trajectory) to a
      // meta_planner::trajectory::Trajectory.
      meta_planner_msgs::Trajectory traj_msg;
      for (size_t jj = 0; jj < srv.res.traj.times.size(); jj++) {
        meta_planner_msgs::PlannerState ps;
        ps.position = ToPosition(srv.res.traj.states[jj], ii);
        ps.previous_planner_id = jj;  // Is this right for jj=0?
        ps.previous_planner_state = srv.res.traj.states[jj];
        ps.next_planner_id = jj;  // Is this right for last jj?
        ps.next_planner_state = srv.res.traj.states[jj];

        traj_msg.states.push_back(ps);
        traj_msg.times.push_back(srv.res.traj.times[jj]);
      }

      traj = Trajectory(traj_msg);

      if (!traj.Empty()) {
        // When we succeed...
        // If we just planned with a more cautious planner than the one used
        // by the nearest neighbor, do a 1-step backtrack.
        if (!is_root && ii > neighbor->planner_id) {
          // Clone the neighbor.
          const Vector3d jittered(neighbor->point_(0) + 1e-4,
                                  neighbor->point_(1) + 1e-4,
                                  neighbor->point_(2) + 1e-4);

          // Find start time of backtrack trajectory.
          ROS_ASSERT(!neighbor->traj.Empty());
          const double backtrack_start_time = neighbor->traj.FirstTime();

          // Get id of planner used for transition.
          const size_t transition_planner_id =
              ToFlatIndex(neighbor->planner_id, ii);

          // Attempt to plan using the transition planner.
          // TODO

          // Create the cloned waypoint using the backtrack trajectory.
          Waypoint::ConstPtr clone = Waypoint::Create(
              jittered, neighbor->state_, transition_planner_id, backtrack_traj,
              neighbor->parent_);

          // Swap out the control value function in the neighbor's
          // trajectory
          // and update time stamps accordingly.
          clone->traj_->ExecuteSwitch(value_used, best_time_srv_);

          // Insert the clone.
          tree.Insert(clone, false);

          // Adjust the time stamps for the new trajectory to occur after
          // the
          // updated neighbor's trajectory.
          traj->ResetStartTime(clone->traj_->LastTime());

          // Neighbor is now clone.
          neighbor = clone;
        }

        break;
      }
    }

    // Check if we could found a trajectory to this sample.
    if (traj == nullptr) continue;

    // Insert the sample.
    const Waypoint::ConstPtr waypoint =
        Waypoint::Create(sample, value_used, traj, neighbor);

    tree.Insert(waypoint, false);

    // (5) Try to connect to the goal point.
    Trajectory::Ptr goal_traj;
    ValueFunctionId goal_value_used;
    const size_t planner_used_id = value_used / 2;

    if ((sample - stop).norm() <= max_connection_radius_) {
      for (size_t ii = 0; ii < std::min(planner_used_id + 2, planners_.size());
           ii++) {
        const Planner::ConstPtr planner = planners_[ii];
        goal_value_used = planner->GetIncomingValueFunction();

        // We are never gonna need to switch if this succeeds.
        // Plan using 10% of the available total runtime.
        // NOTE! This is just a heuristic and could easily be changed.
        goal_traj =
            planner->Plan(sample, stop, traj->LastTime(), 0.1 * max_runtime_);

        if (goal_traj != nullptr) {
          // When we succeed... don't need to clone because waypoint has no
          // kids.
          // If we just planned with a more cautious planner than the one used
          // by the nearest neighbor, do a 1-step backtrack.
          if (ii > neighbor_planner_id) {
            // Swap out the control value function in the neighbor's
            // trajectory
            // and update time stamps accordingly.
            waypoint->traj_->ExecuteSwitch(goal_value_used, best_time_srv_);

            // Adjust the time stamps for the new trajectory to occur after
            // the
            // updated neighbor's trajectory.
            goal_traj->ResetStartTime(waypoint->traj_->LastTime());
          }

          break;
        }
      }
    }

    // (6) If this sample was connected to the goal, update the tree terminus.
    if (goal_traj != nullptr) {
      // Connect to the goal.
      // NOTE: the first point in goal_traj coincides with the last point in
      // traj, but when we merge the two trajectories the std::map insertion
      // rules will prevent duplicates.
      const Waypoint::ConstPtr goal =
          Waypoint::Create(stop, value_used, goal_traj, waypoint);

      tree.Insert(goal, true);

      // Mark that we've found a valid trajectory.
      found = true;
    }
  }

  if (found) {
    // Get the best (fastest) trajectory out of the tree.
    const Trajectory::ConstPtr best = tree.BestTrajectory();
    ROS_INFO("%s: Publishing trajectory of length %zu.", name_.c_str(),
             best->Size());

    traj_ = best;
    traj_pub_.publish(best->ToRosMessage());
    return true;
  }

  return false;
}

template <typename S>
bool MetaPlanner<S>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "meta_planner");

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

template <typename S>
bool MetaPlanner<S>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Random seed.
  int seed = 0;
  if (!nl.getParam("random/seed", seed)) return false;
  seed_ = static_cast<unsigned int>(seed);

  // Meta planning parameters.
  if (!nl.getParam("max_runtime", max_runtime_)) return false;
  if (!nl.getParam("max_connection_radius", max_connection_radius_))
    return false;

  // Number of planners. NOTE: this is the square root of the number of
  // planner
  // services we expect to have.
  int num_planners = 1;
  if (!nl.getParam("num_planners", num_planners)) return false;
  num_planners_ = static_cast<size_t>(num_planners);

  // Service names.
  if (!nl.getParam("srv/planners", planner_srv_names_)) return false;
  if (planner_srv_names_.size() != num_planners_ * num_planners) {
    ROS_ERROR("%s: wrong number of planner service names.", name_.c_str());
    return false;
  }

  // Planner state types.
  if (!nl.getParam("planner_state_types", planner_state_types_)) return false;
  if (planner_state_types_.size() != num_planners_ * num_planners_) {
    ROS_ERROR("%s: wrong number of planner state types.", name_.c_str());
    return false;
  }

  // Topics and frame ids.
  if (!nl.getParam("topics/traj", traj_topic_)) return false;
  if (!nl.getParam("topics/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;

  return true;
}

template <typename S>
bool MetaPlanner<S>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Services.
  for (const auto& name : planner_srv_names_) {
    ros::service::waitForService(name.c_str());
    planner_srvs_.push_back(
        nl.serviceClient<fastrack_srvs::Replan>(name.c_str(), true));
  }

  // Subscribers.
  request_traj_sub_ =
      nl.subscribe(request_traj_topic_.c_str(), 1,
                   &MetaPlanner::RequestTrajectoryCallback, this);

  // Actual publishers.
  traj_pub_ = nl.advertise<meta_planner_msgs::Trajectory>(traj_topic_.c_str(),
                                                          1, false);

  return true;
}

template <typename S>
void MetaPlanner<S>::RequestTrajectoryCallback(
    const meta_planner_msgs::ReplanRequest::ConstPtr& msg) {
  ROS_INFO("%s: Recomputing trajectory.", name_.c_str());
  const ros::Time current_time = ros::Time::now();

  // Unpack the message.
  const fastrack_msgs::State start = msg->start;
  const fastrack_msgs::State goal = msg->goal;
  const double start_time = msg->start_time;
  const size_t initial_planner_id = msg->initial_planner_id;

  if (!Plan(start, goal, start_time, initial_planner_id)) {
    ROS_ERROR("%s: MetaPlanner failed. Please come again.", name_.c_str());
    return;
  }

  ROS_INFO("%s: MetaPlanner succeeded after %2.5f seconds.", name_.c_str(),
           (ros::Time::now() - current_time).toSec());
}

}  //\namespace planning
}  //\namespace meta

#endif
