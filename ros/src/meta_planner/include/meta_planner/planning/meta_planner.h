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
  fastrack_msgs::State ToPlannerStateMsg(const S& tracker_x,
                                         size_t planner_id) const;

  // Convert from a planner state msg to a geometric position.
  Vector3d ToPosition(const fastrack_msgs::State& planner_x,
                      size_t planner_id) const;

  // List of planner services.
  // NOTE: this is actually a flattened matrix of planners, since
  // we're going to have separate planners depending on whether they're using
  // TEBs or SSBs in order to keep the fastrack interface intact.
  size_t num_planners_;
  std::vector<ros::ServiceClient> planner_srvs_;
  std::vector<std::string> planner_srv_names_;

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
fastrack_msgs::State MetaPlanner<S>::ToPlannerStateMsg(const S& tracker_x,
                                                       size_t planner_id) const;

template <typename S>
Vector3d MetaPlanner<S>::ToPosition(const fastrack_msgs::State& planner_x,
                                    size_t planner_id) const;

template <typename S>
bool MetaPlanner<S>::Plan(const fastrack_msgs::State& start,
                          const fastrack_msgs::State& goal, double start_time,
                          size_t initial_planner_id) {
  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time current_time = ros::Time::now();
  WaypointTree tree(start, initial_planner_id, start_time);

  bool found = false;
  while ((ros::Time::now() - current_time).toSec() < max_runtime_) {
    // (2) Sample a new point in the state space.
    const S sample = S::Sample();

    // (3) Find the nearest neighbor.
    constexpr size_t kNumNeighbors = 1;
    const std::vector<Waypoint::ConstPtr> neighbors =
        tree.KnnSearch(sample, kNumNeighbors);

    // Throw out this sample if too far from the nearest point.
    if (neighbors.size() != kNumNeighbors ||
        (neighbors[0]->point_ - sample).norm() > max_connection_radius_)
      continue;

    Waypoint::ConstPtr neighbor = neighbors[0];

    // Extract value function and corresponding planner ID from last waypoint.
    // If value is null, (i.e. at root) then set to planners_.size() since
    // any planner is valid from the root. Convert value ID to planner ID
    // by dividing by 2 since each planner has two value functions.
    const Trajectory::ConstPtr neighbor_traj = neighbor->traj_;
    const ValueFunctionId neighbor_val = neighbor->value_;

    const size_t neighbor_planner_id = neighbor_val / 2;

    // (4) Plan a trajectory (starting with the most aggressive planner and
    // ending
    // with the next-most cautious planner).
    Trajectory::Ptr traj;
    ValueFunctionId value_used;
    for (size_t ii = 0;
         ii < std::min(neighbor_planner_id + 2, planners_.size()); ii++) {
      const Planner::ConstPtr planner = planners_[ii];

      value_used = planner->GetIncomingValueFunction();
      const ValueFunctionId possible_next_value =
          planner->GetOutgoingValueFunction();

      // Make sure switching distance server is up.
      if (!switching_distance_srv_) {
        ROS_WARN("%s: Switching distance server disconnected.", name_.c_str());

        ros::NodeHandle nl;
        switching_distance_srv_ =
            nl.serviceClient<value_function_srvs::GuaranteedSwitchingDistance>(
                switching_distance_name_.c_str(), true);
        return false;
      }

      // Get the tracking bound for this planner.
      double switch_x = 0.0;
      double switch_y = 0.0;
      double switch_z = 0.0;

      value_function_srvs::GuaranteedSwitchingDistance d;
      d.request.from_id = value_used;
      d.request.to_id = possible_next_value;
      if (!switching_distance_srv_.call(d))
        ROS_ERROR("%s: Error calling switching distance server.",
                  name_.c_str());
      else {
        switch_x = d.response.x;
        switch_y = d.response.y;
        switch_z = d.response.z;
      }

      // Since we might always end up switching, make sure this point
      // is not closer than the guaranteed switching distance.
      // NOTE! This enforces backtracking only one planner at a time.
      // In full generality, we would just need to replace possible_next_value
      // with the most cautious value.
      if (std::abs(neighbor->point_(0) - sample(0)) < switch_x &&
          std::abs(neighbor->point_(1) - sample(1)) < switch_y &&
          std::abs(neighbor->point_(2) - sample(2)) < switch_z)
        continue;

      // Plan using 10% of the available total runtime.
      // NOTE! This is just a heuristic and could easily be changed.
      const double time =
          (neighbor_traj == nullptr) ? start_time : neighbor_traj->LastTime();

      traj = planner->Plan(neighbor->point_, sample, time, 0.1 * max_runtime_);

      if (traj != nullptr) {
        // When we succeed...
        // If we just planned with a more cautious planner than the one used
        // by the nearest neighbor, do a 1-step backtrack.
        if (ii > neighbor_planner_id) {
#if 0
          std::cout << "Switched from planner " << neighbor_planner_id
                    << " with value id " << neighbor_val->Id()
                    << " to planner " << ii
                    << " with value id " << value_used->Id() << std::endl;
#endif
          // Clone the neighbor.
          const Vector3d jittered(neighbor->point_(0) + 1e-4,
                                  neighbor->point_(1) + 1e-4,
                                  neighbor->point_(2) + 1e-4);

          const double time = (neighbor_traj == nullptr)
                                  ? start_time
                                  : neighbor_traj->FirstTime();

          if (time <= start_time + 1e-8) {
            ROS_INFO_THROTTLE(1.0, "%s: Tried to clone the root.",
                              name_.c_str());

            // Didn't really succeed. Can't clone the root in general.
            traj = nullptr;
          } else {
            Waypoint::ConstPtr clone = Waypoint::Create(
                jittered, value_used, Trajectory::Create(neighbor_traj, time),
                neighbor->parent_);

            // Swap out the control value function in the neighbor's trajectory
            // and update time stamps accordingly.
            clone->traj_->ExecuteSwitch(value_used, best_time_srv_);

            // Insert the clone.
            tree.Insert(clone, false);

            // Adjust the time stamps for the new trajectory to occur after the
            // updated neighbor's trajectory.
            traj->ResetStartTime(clone->traj_->LastTime());

            // Neighbor is now clone.
            neighbor = clone;
          }
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
            // Swap out the control value function in the neighbor's trajectory
            // and update time stamps accordingly.
            waypoint->traj_->ExecuteSwitch(goal_value_used, best_time_srv_);

            // Adjust the time stamps for the new trajectory to occur after the
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

  // Number of planners. NOTE: this is the square root of the number of planner
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
