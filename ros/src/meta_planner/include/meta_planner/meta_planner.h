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
// Defines the MetaPlanner class. The MetaPlanner samples random points in
// the state space and then spawns off different Planners to plan Trajectories
// between these points (RRT-style).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_META_PLANNER_H
#define META_PLANNER_META_PLANNER_H

#include <meta_planner/waypoint_tree.h>
#include <meta_planner/waypoint.h>
#include <meta_planner/ompl_planner.h>
#include <meta_planner/environment.h>
#include <value_function/near_hover_quad_no_yaw.h>
#include <utils/types.h>
#include <utils/uncopyable.h>
#include <demo/balls_in_box.h>

#include <meta_planner_msgs/Trajectory.h>
#include <meta_planner_msgs/TrajectoryRequest.h>
#include <meta_planner_msgs/SensorMeasurement.h>
#include <crazyflie_msgs/PositionVelocityStateStamped.h>

#include <value_function_srvs/TrackingBoundBox.h>
#include <value_function_srvs/GeometricPlannerTime.h>
#include <value_function_srvs/GuaranteedSwitchingTime.h>
#include <value_function_srvs/GuaranteedSwitchingDistance.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <limits>

namespace meta {

class MetaPlanner : private Uncopyable {
public:
  ~MetaPlanner() {}
  explicit MetaPlanner()
    : in_flight_(false),
      reached_goal_(false),
      been_updated_(false),
      initialized_(false) {}

  // Initialize this class from a ROS node.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing state updates.
  void StateCallback(
    const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Callback for processing sensor measurements.
  void SensorCallback(
    const meta_planner_msgs::SensorMeasurement::ConstPtr& msg);

  // Callback for updating in flight status.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }

  // Callback to handle requests for new trajectory.
  void RequestTrajectoryCallback(
    const meta_planner_msgs::TrajectoryRequest::ConstPtr& msg);

  // Callback to handle new waypoints.
  void WaypointCallback(const geometry_msgs::Vector3::ConstPtr& msg);

  // Plan a trajectory from the given start to stop points, beginning at the
  // specified start time. Auto-publishes the result and returns whether
  // meta planning was successful.
  bool Plan(const Vector3d& start, const Vector3d& stop, double start_time);

  // Hover at the end of the current trajectory.
  void Hover();

  // Dynamics.
  NearHoverQuadNoYaw::ConstPtr dynamics_;

  // Remember the last trajectory we sent.
  Trajectory::ConstPtr traj_;

  // List of planners.
  std::vector<Planner::ConstPtr> planners_;
  size_t num_value_functions_;

  // Sequence of waypoints to go to. Plan each trajectory to the next one in line.
  std::list<Vector3d> waypoints_;

  // Current position, with flag for whether been updated since initialization.
  Vector3d position_;
  bool been_updated_;

  // Spaces and dimensions.
  size_t state_dim_;
  size_t control_dim_;
  BallsInBox::Ptr space_;
  unsigned int seed_;

  std::vector<double> state_upper_;
  std::vector<double> state_lower_;
  std::vector<double> control_upper_;
  std::vector<double> control_lower_;

  // Max time to spend searching for an optimal path.
  double max_runtime_;

  // Maximum distance between waypoints.
  double max_connection_radius_;

  // Services and names.
  ros::ServiceClient bound_srv_;
  ros::ServiceClient best_time_srv_;
  ros::ServiceClient switching_time_srv_;
  ros::ServiceClient switching_distance_srv_;

  std::string bound_name_;
  std::string best_time_name_;
  std::string switching_time_name_;
  std::string switching_distance_name_;

  // Publishers/subscribers and related topics.
  ros::Publisher traj_pub_;
  ros::Publisher env_pub_;
  ros::Publisher trigger_replan_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber sensor_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber request_traj_sub_;
  ros::Subscriber in_flight_sub_;

  std::string traj_topic_;
  std::string env_topic_;
  std::string state_topic_;
  std::string sensor_topic_;
  std::string waypoint_topic_;
  std::string request_traj_topic_;
  std::string trigger_replan_topic_;
  std::string in_flight_topic_;

  // Frames.
  std::string fixed_frame_id_;

  // Are we in flight?
  bool in_flight_;

  // Have we reached the goal?
  bool reached_goal_;

  // Initialization and naming.
  bool initialized_;
  std::string name_;
};

} //\namespace meta

#endif
