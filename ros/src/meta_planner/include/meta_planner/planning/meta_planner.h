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
#include <meta_planner/trajectory/trajectory.h>

#include <fastrack_msgs/State.h>
#include <fastrack_msgs/Trajectory.h>
#include <meta_planner_msgs/ReplanRequest.h>
#include <meta_planner_msgs/SensorMeasurement.h>
#include <meta_planner_msgs/Trajectory.h>

#include <fastrack_srvs/Replan.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <queue>
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

  // Convert planner state msg to position in 3D.
  Vector3d ToPosition(const fastrack_msgs::State& msg, size_t planner_id) const;

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
  // Assumes start and goal states are in the tracker's state space S.
  bool Plan(const fastrack_msgs::State& start, const fastrack_msgs::State& goal,
            double start_time, size_t initial_planner_id);

  // Convert from a tracker state (S) to a planner state fastrack msg.
  fastrack_msgs::State ToPlannerStateMsg(const S& x_in_tracker_space,
                                         size_t planner_id) const;

  // Convert between two planner state types.
  fastrack_msgs::State ConvertPlannerStateMsgs(
      const fastrack_msgs::State& planner1_x, size_t planner1_id,
      size_t planner2_id) const;

  // Convert from fastrack_msgs::Trajectory to meta_planner_msgs::Trajectory.
  meta_planner_msgs::Trajectory ToMetaTrajectoryMsg(
      const fastrack_msgs::Trajectory& msg, size_t planner_id) const;

  // List of planner services.
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

// Custom comparitor for trajectories.
struct TrajectoryComparitor {
  static Vector3d goal_position_;
  static double goal_distance_;

  bool operator()(const Trajectory& lhs, const Trajectory& rhs) {
    // HACK! Assuming first 3 dimensions are position.
    std::vector<double> lhs_last_state = lhs.LastState().x;
    std::vector<double> rhs_last_state = rhs.LastState().x;

    const double lhs_end_dist_sq =
        (Vector3d(lhs_last_state[0], lhs_last_state[1], lhs_last_state[2]) -
         goal_position_)
            .squaredNorm();
    const double rhs_end_dist_sq =
        (Vector3d(rhs_last_state[0], rhs_last_state[1], rhs_last_state[2]) -
         goal_position_)
            .squaredNorm();

    // Compute total distance covered toward goal.
    const double lhs_dist = std::sqrt(lhs_end_dist_sq) - goal_distance_;
    const double rhs_dist = std::sqrt(rhs_end_dist_sq) - goal_distance_;

    // Actual comparison by average speed toward goal.
    return lhs_dist / lhs.Duration() > rhs_dist / rhs.Duration();
  }
};  // TrajectoryComparitor

// ---------------------------- IMPLEMENTATION --------------------------- //

Vector3d TrajectoryComparitor::goal_position_ = Vector3d::Zero();
double TrajectoryComparitor::goal_distance_ =
    std::numeric_limits<double>::infinity();

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

  if (planner_state_types_[planner_id] == "PositionVelocity") {
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

  if (planner_state_types_[planner_id] == "PositionVelocity") {
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

  const auto& type1 = planner_state_types_[planner1_id];
  const auto& type2 = planner_state_types_[planner2_id];

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
  // (1) Keep track of the current time.
  const ros::Time current_time = ros::Time::now();

  // Unpack start and goal position.
  const S start_state = S(start);
  const Vector3d start_position = start_state.Position();
  const Vector3d goal_position = S(goal).Position();
  const double goal_distance_sq =
      (start_position - goal_position).squaredNorm();
  const double goal_distance = std::sqrt(goal_distance_sq);

  for (size_t planner_id = 0; planner_id < num_planners_; planner_id++) {
    // Keep track of any solutions and how close they come to the goal.
    TrajectoryComparitor::goal_position_ = goal_position;
    TrajectoryComparitor::goal_distance_ = goal_distance;
    std::priority_queue<Trajectory, std::vector<Trajectory>,
                        TrajectoryComparitor>
        candidates;

    while (ros::Time::now().toSec() - current_time.toSec() <
           max_runtime_ / num_planners_) {
      // (2) Sample a new point in the state space.
      const S sample = S::Sample();
      Vector3d sample_pos = sample.Position();

      // Reject this sample if it's not close enough to the goal.
      // NOTE: as a heuristic, we reject if not < 0.9 * start distance to
      // goal.
      if ((sample_pos - goal_position).squaredNorm() < 0.81 * goal_distance_sq)
        continue;

      // (3) Plan a trajectory (starting with the first planner and
      // ending with the last planner).
      // Convert start and goal to a fastrack_msgs::State in the planner's
      // state space.
      const fastrack_msgs::State goal_planner_x =
          ToPlannerStateMsg(sample, planner_id);
      const fastrack_msgs::State start_planner_x =
          ToPlannerStateMsg(S(start), planner_id);

      fastrack_srvs::Replan srv;
      srv.request.req.start = start_planner_x;
      srv.request.req.goal = goal_planner_x;
      srv.request.req.start_time = start_time;

      if (!planner_srvs_[planner_id].call(srv)) {
        ROS_ERROR_THROTTLE(
            1.0, "%s: Server failed for planner %zu on service %s",
            name_.c_str(), planner_id, planner_srv_names_[planner_id].c_str());
        continue;
      }

      // Convert service response (fastrack_msgs::Trajectory) to a
      // meta_planner::trajectory::Trajectory.
      const Trajectory traj(ToMetaTrajectoryMsg(srv.response.traj, planner_id));

      // Check if we found a trajectory to this sample.
      if (traj.Empty()) {
        ROS_WARN_THROTTLE(1.0, "%s: Planned trajectory is empty.",
                          name_.c_str());
        continue;
      }
      candidates.push(traj);
    }

    if (!candidates.empty()) {
      // Get the best (fastest) trajectory out of the tree.
      const Trajectory best = candidates.top();
      ROS_INFO(
          "%s: Publishing trajectory of length %zu, planned with planner "
          "%zu.",
          name_.c_str(), best.Size(), planner_id);

      traj_pub_.publish(best.ToRos());
      return true;
    }
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
