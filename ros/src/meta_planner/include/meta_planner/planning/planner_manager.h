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
// Defines the PlannerManager class, which listens for new trajectory
// messages and, on a timer, repeatedly queries the current trajectory and
// publishes the corresponding reference.
//
// The PlannerManager is also responsible for requesting new plans.
// This base class only calls the planner once upon takeoff; as needs will vary
// derived classes may add further functionality such as receding
// horizon planning.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_PLANNING_PLANNER_MANAGER_H
#define META_PLANNER_PLANNING_PLANNER_MANAGER_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>
#include <meta_planner/trajectory/trajectory.h>

#include <fastrack_msgs/ReplanRequest.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace meta {
namespace planning {

using meta_planner::trajectory::Trajectory;

class PlannerManager : private fastrack::Uncopyable {
 public:
  virtual ~PlannerManager() {}
  PlannerManager()
      : ready_(false),
        waiting_for_traj_(false),
        serviced_updated_env_(false),
        initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

 protected:
  // Load parameters and register callbacks. These may be overridden, however
  // derived classes should still call these functions.
  virtual bool LoadParameters(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // If not already waiting, request a new trajectory that starts along the
  // current trajectory at the state corresponding to when the planner will
  // return, and ends at the goal location. This may be overridden by derived
  // classes with more specific replanning needs.
  virtual void MaybeRequestTrajectory();

  // Callback for applying tracking controller. This may be overridden, however
  // derived classes should still call thhis function.
  virtual void TimerCallback(const ros::TimerEvent& e);

  // Create and publish a marker at goal state.
  virtual void VisualizeGoal() const;

  // Callback for processing trajectory updates.
  void TrajectoryCallback(const meta_planner_msgs::Trajectory::ConstPtr& msg);

  // Is the system ready?
  void ReadyCallback(const std_msgs::Empty::ConstPtr& msg) { ready_ = true; }

  // Generate a new trajectory request when environment has been updated.
  void UpdatedEnvironmentCallback(const std_msgs::Empty::ConstPtr& msg);

  // Current trajectory.
  Trajectory traj_;

  // Planner runtime -- how long does it take for the planner to run.
  double planner_runtime_;

  // Are we waiting for a new trajectory?
  bool waiting_for_traj_;

  // Have we serviced the most recent updated environment callback?
  bool serviced_updated_env_;

  // Start/goal states.
  fastrack_msgs::State start_;
  fastrack_msgs::State goal_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // Publishers/subscribers and related topics.
  ros::Publisher goal_pub_;
  ros::Publisher ref_pub_;
  ros::Publisher replan_request_pub_;
  ros::Publisher traj_vis_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber ready_sub_;
  ros::Subscriber updated_env_sub_;

  std::string goal_topic_;
  std::string ref_topic_;
  std::string replan_request_topic_;
  std::string traj_vis_topic_;
  std::string traj_topic_;
  std::string ready_topic_;
  std::string updated_env_topic_;

  // Frames of reference for publishing markers.
  std::string fixed_frame_;
  std::string planner_frame_;

  // Transform broadcaster for planner position.
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Are we in flight?
  bool ready_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

}  //\namespace planning
}  //\namespace meta

#endif
