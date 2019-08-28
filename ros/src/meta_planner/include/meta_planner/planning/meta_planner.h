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
#include <fastrack/state/state.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/planning/waypoint.h>
#include <meta_planner/planning/waypoint_tree.h>

#include <fastrack_msgs/State.h>
#include <fastrack_msgs/Trajectory.h>
#include <meta_planner_msgs/ReplanRequest.h>
#include <meta_planner_msgs/SensorMeasurement.h>
#include <meta_planner_msgs/Trajectory.h>

#include <fastrack_srvs/Replan.h>

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

  // Try to connect the waypoint to the given goal state. Returns ptr to
  // waypoint inserted (or nullptr if failed).
  Waypoint::ConstPtr ConnectAndBacktrack(const Waypoint::ConstPtr& start,
                                         const S& goal, double start_time,
                                         bool is_terminus, WaypointTree* tree);

  // Convert from a tracker state (S) to a planner state fastrack msg.
  fastrack_msgs::State ToPlannerStateMsg(const S& x_in_tracker_space,
                                         size_t planner_id) const;

  // Convert planner state msg to position in 3D.
  Vector3d ToPosition(const fastrack_msgs::State& msg, size_t planner_id) const;

  // Convert between two planner state types.
  fastrack_msgs::State ConvertPlannerStateMsgs(
      const fastrack_msgs::State& planner1_x, size_t planner1_id,
      size_t planner2_id) const;

  // Convert from fastrack_msgs::Trajectory to meta_planner_msgs::Trajectory.
  meta_planner_msgs::Trajectory ToMetaTrajectoryMsg(
      const fastrack_msgs::Trajectory& msg, size_t planner_id) const;

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

  // Random seed.
  size_t seed_;

  // Are we in flight?
  bool in_flight_;

  // Initialization and naming.
  bool initialized_;
  std::string name_;
};

// ---------------------------- IMPLEMENTATION --------------------------- //

template <typename S>
meta_planner_msgs::Trajectory MetaPlanner<S>::ToMetaTrajectoryMsg(
    const fastrack_msgs::Trajectory& msg, size_t planner_id) const {
  meta_planner_msgs::Trajectory traj_msg;
  for (size_t jj = 0; jj < msg.times.size(); jj++) {
    meta_planner_msgs::PlannerState ps;
    const Vector3d pos = ToPosition(msg.states[jj], planner_id);

    ps.position.x = pos.x();
    ps.position.y = pos.y();
    ps.position.z = pos.z();
    ps.previous_planner_id = planner_id;  // Is this right for first jj?
    ps.previous_planner_state = msg.states[jj];
    ps.next_planner_id = planner_id;  // Is this right for last jj?
    ps.next_planner_state = msg.states[jj];

    traj_msg.states.push_back(ps);
    traj_msg.times.push_back(msg.times[jj]);
  }

  return traj_msg;
}

template <typename S>
fastrack_msgs::State MetaPlanner<S>::ToPlannerStateMsg(
    const S& x_in_tracker_space, size_t planner_id) const {
  fastrack_msgs::State msg;

  if (planner_state_types_[ToFlatIndex(planner_id, planner_id)] ==
      "PositionVelocity") {
    PositionVelocity x_in_planner_space(x_in_tracker_space);
    msg = x_in_planner_space.ToRos();
  }
  // TODO: can add more state conversions later if needed.
  else
    ROS_FATAL("%s: You dummy. This is not a valid state type.", name_.c_str());

  return msg;
}

template <typename S>
Vector3d MetaPlanner<S>::ToPosition(const fastrack_msgs::State& msg,
                                    size_t planner_id) const {
  Vector3d position;

  if (planner_state_types_[ToFlatIndex(planner_id, planner_id)] ==
      "PositionVelocity") {
    PositionVelocity planner_x(msg);
    position = planner_x.Position();
  }
  // TODO: can add more state conversions later if needed.
  else
    ROS_FATAL("%s: You dummy. This is not a valid state type.", name_.c_str());

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
Waypoint::ConstPtr MetaPlanner<S>::ConnectAndBacktrack(
    const Waypoint::ConstPtr& start, const S& goal, double start_time,
    bool is_terminus, WaypointTree* tree) {
  ROS_ASSERT(tree != nullptr);
  ROS_ASSERT(start.get() != nullptr);

  // Check if close enough.
  if ((start->point_ - goal.Position()).norm() > max_connection_radius_)
    return nullptr;

  // Is the start the root.
  const bool is_root = (start->parent_ == nullptr);

  Trajectory traj;
  for (size_t ii = 0; ii < num_planners_; ii++) {
    // Convert this to a fastrack_msgs::State in the planner's state space.
    const fastrack_msgs::State goal_planner_x = ToPlannerStateMsg(goal, ii);

    // Also convert the start's state to this (ii'th) planner state.
    const fastrack_msgs::State start_planner_x =
        ConvertPlannerStateMsgs(start->state_, start->planner_id_, ii);

    const double time = (is_root) ? start_time : start->traj_.LastTime();

    /*
    std::cout << "Connect: Is root: " << is_root << std::endl;
    std::cout << "Connect: Start's position: " << start->point_.transpose() <<
    std::endl;
    std::cout << "Connect: Start's trajectory length: " << start->traj_.Size()
              << std::endl;
    std::cout << "Connect invoked with start time: " << start_time << std::endl;
    std::cout << "Connecting starting at time " << time << std::endl;
    */

    // Call planner ii's service.
    fastrack_srvs::Replan srv;
    srv.request.req.start = start_planner_x;
    srv.request.req.goal = goal_planner_x;
    srv.request.req.start_time = time;

    if (!planner_srvs_[ToFlatIndex(start->planner_id_, ii)].call(srv)) {
      ROS_ERROR_THROTTLE(
          1.0, "%s: Server failed for planner %zu=>%zu on service %s",
          name_.c_str(), start->planner_id_, ii,
          planner_srv_names_[ToFlatIndex(start->planner_id_, ii)].c_str());
      continue;
    }

    // Convert service response (fastrack_msgs::Trajectory) to a
    // meta_planner::trajectory::Trajectory.
    traj = Trajectory(ToMetaTrajectoryMsg(srv.response.traj, ii));
    Waypoint::ConstPtr sample_parent = start;

    // Check if we found a trajectory to this goal.
    if (traj.Empty()) continue;

    // When we succeed...
    // If we just planned with a more cautious planner than the one used
    // by the nearest start, do a 1-step backtrack.
    if (!is_root && ii > start->planner_id_) {
      ROS_INFO_THROTTLE(1.0, "%s: Backtracking from planner %zu.",
                        name_.c_str(), ii);

      // Clone the start.
      const Vector3d jittered(start->point_(0) + 1e-4, start->point_(1) + 1e-4,
                              start->point_(2) + 1e-4);

      // Find start time of backtrack trajectory.
      ROS_ASSERT(!start->traj_.Empty());
      const double backtrack_start_time = start->traj_.FirstTime();

      // Get id of planner used for transition.
      // NOTE: the transition is from start's parent's planner to ii.
      ROS_ASSERT(start->parent_.get() != nullptr);
      const size_t transition_planner_id =
          ToFlatIndex(start->parent_->planner_id_, ii);

      // Attempt to plan using the transition planner.
      fastrack_srvs::Replan backtrack_srv;
      backtrack_srv.request.req.start = start_planner_x;
      backtrack_srv.request.req.goal = goal_planner_x;
      backtrack_srv.request.req.start_time = time;

      if (!planner_srvs_[transition_planner_id].call(backtrack_srv)) {
        ROS_ERROR("%s: Server failed for planner %zu=>%zu.", name_.c_str(),
                  start->parent_->planner_id_, ii);
        continue;
      }

      const Trajectory backtrack_traj(
          ToMetaTrajectoryMsg(backtrack_srv.response.traj, ii));

      // Catch case where planning fails or returns terminal state significantly
      // different from start state.
      if (backtrack_traj.Empty() ||
          std::abs(backtrack_traj.Positions().back().x - start->point_.x()) >
              1e-4 ||
          std::abs(backtrack_traj.Positions().back().y - start->point_.y()) >
              1e-4 ||
          std::abs(backtrack_traj.Positions().back().z - start->point_.z()) >
              1e-4)
        continue;

      // Create the cloned waypoint using the backtrack trajectory.
      Waypoint::ConstPtr clone =
          Waypoint::Create(jittered, start_planner_x, transition_planner_id,
                           backtrack_traj, start->parent_);

      // Insert the clone.
      tree->Insert(clone, false);

      // Adjust the time stamps for the new trajectory to occur after
      // the updated start's trajectory.
      traj.ResetFirstTime(clone->traj_.LastTime());

      // Start is now clone.
      sample_parent = clone;
    }

    // Extract final point in plan.
    const S final_x(traj.LastState());
    const Vector3d final_pos = final_x.Position();

    // Catch nans.
    if (final_x.ToVector().hasNaN()) return nullptr;

    // Error out if this was supposed to be a terminus but we're super far from
    // the desired goal.
    if (is_terminus && (final_pos - goal.Position()).norm() > 0.1)
      return nullptr;

    // Insert the final state in the plan toward the goal.
    const Waypoint::ConstPtr waypoint =
        Waypoint::Create(final_pos, traj.LastState(), ii, traj, sample_parent);

    tree->Insert(waypoint, is_terminus);
    return waypoint;
  }

  return nullptr;
}

template <typename S>
bool MetaPlanner<S>::Plan(const fastrack_msgs::State& start,
                          const fastrack_msgs::State& goal, double start_time,
                          size_t initial_planner_id) {
  std::cout << "Plan invoked with start time " << start_time << std::endl;

  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time current_time = ros::Time::now();
  WaypointTree tree(S(start).Position(), start, initial_planner_id, start_time);

  // Unpack goal position.
  const Vector3d goal_position = S(goal).Position();

  bool found = false;
  while ((ros::Time::now() - current_time).toSec() < max_runtime_) {
    // (2) Sample a new point in the state space.
    const S sample = S::Sample();
    Vector3d sample_pos = sample.Position();

    // (3) Find the nearest neighbor in position space.
    constexpr size_t kNumNeighbors = 1;
    const std::vector<Waypoint::ConstPtr> neighbors =
        tree.KnnSearch(sample_pos, kNumNeighbors);

    if (neighbors.size() != kNumNeighbors) continue;
    Waypoint::ConstPtr neighbor = neighbors[0];

    // Is the neighbor the root.
    const bool is_root = (neighbor->parent_ == nullptr);

    // (4) Plan a trajectory (starting with the first planner and
    // ending with the last planner).
    const double time = (is_root) ? start_time : neighbor->traj_.LastTime();
    //    std::cout << "Sample pos: " << sample_pos.transpose() << std::endl;
    const Waypoint::ConstPtr waypoint =
        ConnectAndBacktrack(neighbor, sample, time, false, &tree);

    if (!waypoint) continue;

    // (5) Try to connect to the goal point.
    const Waypoint::ConstPtr goal_waypoint = ConnectAndBacktrack(
        waypoint, S(goal), waypoint->traj_.LastTime(), true, &tree);

    if (goal_waypoint) {
      // Mark that we've found a valid trajectory.
      found = true;

      std::cout << "second-to-last plan endpoint: "
                << waypoint->point_.transpose() << std::endl;
      std::cout << "last plan startpoint: "
                << goal_waypoint->traj_.Positions()[0].x << ", "
                << goal_waypoint->traj_.Positions()[0].y << ", "
                << goal_waypoint->traj_.Positions()[0].z << std::endl;
      std::cout << "last plan endpoint: "
                << goal_waypoint->traj_.Positions().back().x << ", "
                << goal_waypoint->traj_.Positions().back().y << ", "
                << goal_waypoint->traj_.Positions().back().z << std::endl
                << std::flush;
      //      break;
    }
  }

  if (found) {
    // Get the best (fastest) trajectory out of the tree.
    const Trajectory best = tree.BestTrajectory();
    ROS_INFO("%s: Publishing trajectory of length %zu.", name_.c_str(),
             best.Size());

    const double t = ros::Time::now().toSec();

    traj_pub_.publish(best.ToRos());
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

  fastrack::state::State::Seed(seed_);

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

  // Tracker's state space bounds.
  std::vector<double> lower, upper;
  if (!nl.getParam("state_lower", lower)) return false;
  if (!nl.getParam("state_upper", upper)) return false;
  S::SetBounds(lower, upper);

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
  ROS_INFO("%s: Recomputing trajectory to start at time %f.", name_.c_str(),
           msg->start_time);
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
