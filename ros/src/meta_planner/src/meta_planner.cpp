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

#include <meta_planner/meta_planner.h>
#include <meta_planner/time_varying_a_star.h>

#include <ompl/util/Console.h>

namespace meta {

// Initialize this class from a ROS node.
bool MetaPlanner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "meta_planner");

  // Set the initial position and goal to zero. Position will be updated
  // via a message and goal will be read from the parameter server.
  position_ = Vector3d::Zero();
  goal_ = Vector3d::Zero();

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set control upper/lower bounds as Eigen::Vectors.
  VectorXd control_upper_vec(control_dim_);
  VectorXd control_lower_vec(control_dim_);
  for (size_t ii = 0; ii < control_dim_; ii++) {
    control_upper_vec(ii) = control_upper_[ii];
    control_lower_vec(ii) = control_lower_[ii];
  }

  // Set up dynamics.
  dynamics_ = NearHoverQuadNoYaw::Create(control_lower_vec, control_upper_vec);

  // Initialize state space.
	// TODO THIS IS CHANGED to snakesintesseract
  space_ = SnakesInTesseract::Create();
  if (!space_->Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize SnakesInTesseract.", name_.c_str());
    return false;
  }

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(dynamics_->Puncture(state_lower_vec),
                    dynamics_->Puncture(state_upper_vec));

  space_->Seed(seed_);

  // Create planners.
  for (ValueFunctionId ii = 0; ii < num_value_functions_ - 1; ii += 2) {
    const Planner::Ptr planner =
      TimeVaryingAStar::Create(ii, ii + 1, space_, dynamics_, grid_resolution_, 
        collision_check_resolution_);
    //TimeVaryingRrt::Create(ii, ii+1, space_, dynamics_);
    //OmplPlanner<og::BITstar>::Create(ii, ii + 1, space_, dynamics_);

    if (!planner->Initialize(n)) {
      ROS_ERROR("%s: Failed to initialize planner.", name_.c_str());
      return false;
    }

    planners_.push_back(planner);
  }

  // Set OMPL log level.
  //ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);

  // Publish environment.
  space_->Visualize(env_pub_, fixed_frame_id_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool MetaPlanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Random seed.
  int seed = 0;
  if (!nl.getParam("random/seed", seed)) return false;
  seed_ = static_cast<unsigned int>(seed);

  // Meta planning parameters.
  if (!nl.getParam("max_runtime", max_runtime_))
    return false;
  if (!nl.getParam("max_connection_radius", max_connection_radius_))
    return false;

  int dimension = 1;
  if (!nl.getParam("control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("control/upper", control_upper_)) return false;
  if (!nl.getParam("control/lower", control_lower_)) return false;

  if (control_upper_.size() != control_dim_ ||
      control_lower_.size() != control_dim_) {
    ROS_ERROR("%s: Upper and/or lower bounds are in the wrong dimension.",
              name_.c_str());
    return false;
  }

  // Planner parameters.
  int num_values = 2;
  if (!nl.getParam("planners/num_values", num_values)) return false;
  num_value_functions_ = static_cast<size_t>(num_values);

  if (num_value_functions_ % 2 != 0) {
    ROS_ERROR("%s: Must provide an even number of value functions.",
              name_.c_str());
    return false;
  }

  if (!nl.getParam("planners/collision_check_resolution", 
    collision_check_resolution_)) return false;

  if (!nl.getParam("planners/grid_resolution", grid_resolution_)) return false;

  // State space parameters.
  if (!nl.getParam("state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("state/upper", state_upper_)) return false;
  if (!nl.getParam("state/lower", state_lower_)) return false;

  // Goal position.
  double goal_x, goal_y, goal_z;
  if (!nl.getParam("goal/x", goal_x)) return false;
  if (!nl.getParam("goal/y", goal_y)) return false;
  if (!nl.getParam("goal/z", goal_z)) return false;
  goal_ = Vector3d(goal_x, goal_y, goal_z);

  // Service names.
  if (!nl.getParam("srv/tracking_bound", bound_name_)) return false;
  if (!nl.getParam("srv/best_time", best_time_name_)) return false;
  if (!nl.getParam("srv/switching_time", switching_time_name_)) return false;
  if (!nl.getParam("srv/switching_distance", switching_distance_name_))
    return false;

  // Topics and frame ids.
  if (!nl.getParam("topics/sensor", sensor_topic_)) return false;
  if (!nl.getParam("topics/vis/known_environment", env_topic_)) return false;
  if (!nl.getParam("topics/traj", traj_topic_)) return false;
  if (!nl.getParam("topics/state", state_topic_)) return false;
  if (!nl.getParam("topics/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("topics/trigger_replan", trigger_replan_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;

  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;

  return true;
}

// Register callbacks.
bool MetaPlanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Services.
  bound_srv_ = nl.serviceClient<value_function::TrackingBoundBox>(
    bound_name_.c_str(), true);

  best_time_srv_ = nl.serviceClient<value_function::GeometricPlannerTime>(
    best_time_name_.c_str(), true);

  switching_time_srv_ = nl.serviceClient<value_function::GuaranteedSwitchingTime>(
    switching_time_name_.c_str(), true);

  switching_distance_srv_ = nl.serviceClient<value_function::GuaranteedSwitchingDistance>(
    switching_distance_name_.c_str(), true);

  // Subscribers.
  sensor_sub_ = nl.subscribe(
    sensor_topic_.c_str(), 1, &MetaPlanner::SensorCallback, this);

  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &MetaPlanner::StateCallback, this);

  request_traj_sub_ = nl.subscribe(
    request_traj_topic_.c_str(), 1, &MetaPlanner::RequestTrajectoryCallback, this);

  in_flight_sub_ = nl.subscribe(
    in_flight_topic_.c_str(), 1, &MetaPlanner::InFlightCallback, this);

  // Visualization publisher(s).
  env_pub_ = nl.advertise<visualization_msgs::Marker>(
    env_topic_.c_str(), 1, false);

  // Triggering a replan event.
  trigger_replan_pub_ = nl.advertise<std_msgs::Empty>(
    trigger_replan_topic_.c_str(), 1, false);

  // Actual publishers.
  traj_pub_ = nl.advertise<meta_planner_msgs::Trajectory>(
    traj_topic_.c_str(), 1, false);

  return true;
}

// Callback for processing state updates.
void MetaPlanner::StateCallback(
  const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg) {
  position_(0) = msg->state.x;
  position_(1) = msg->state.y;
  position_(2) = msg->state.z;

  been_updated_ = true;
}

// Callback for processing sensor measurements. Replan trajectory.
void MetaPlanner::
SensorCallback(const meta_planner_msgs::SensorMeasurement::ConstPtr& msg) {
  if (!in_flight_)
    return;

  bool unseen_obstacle = false;

  for (size_t ii = 0; ii < msg->num_obstacles; ii++) {
    const double radius = msg->radii[ii];
    const Vector3d point(msg->positions[ii].x,
                         msg->positions[ii].y,
                         msg->positions[ii].z);

		// TODO should remove this with the new planner?
    // Check if our version of the map has already seen this point.
    if (!(space_->IsObstacle(point, radius))) {
      space_->AddObstacle(point, radius);
      unseen_obstacle = true;
    }
  }

  if (unseen_obstacle) {
    // Trigger a replan.
    trigger_replan_pub_.publish(std_msgs::Empty());

    // Publish environment.
    space_->Visualize(env_pub_, fixed_frame_id_);
  }
}

// Callback to handle requests for new trajectory.
void MetaPlanner::RequestTrajectoryCallback(
  const meta_planner_msgs::TrajectoryRequest::ConstPtr& msg) {
  // Only plan if position has been updated.
  if (!been_updated_)
    return;

  ROS_INFO("%s: Recomputing trajectory.", name_.c_str());
  const ros::Time current_time = ros::Time::now();

  // Unpack msg.
  const double start_time = msg->start_time;

  VectorXd start_state(msg->start_state.dimension);
  for (size_t ii = 0; ii < start_state.size(); ii++)
    start_state(ii) = msg->start_state.state[ii];

  const Vector3d start_position = dynamics_->Puncture(start_state);

  // Make sure bound server is up.
  if (!bound_srv_) {
    ROS_WARN("%s: Tracking bound server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    bound_srv_ = nl.serviceClient<value_function::TrackingBoundBox>(
      bound_name_.c_str(), true);
    return;
  }

  // Get the tracking bound for this planner.
  double bound_x = 0.0;
  double bound_y = 0.0;
  double bound_z = 0.0;

  value_function::TrackingBoundBox b;
  b.request.id = planners_.back()->GetOutgoingValueFunction();
  if (!bound_srv_.call(b))
    ROS_ERROR("%s: Error calling tracking bound server.", name_.c_str());
  else {
    bound_x = b.response.x;
    bound_y = b.response.y;
    bound_z = b.response.z;
  }

  // Check if the start position is close to the goal. If so, just return
  // a hover trajectory at the goal (assuming the least aggressive planner).
  if (reached_goal_ ||
      (std::abs(start_position(0) - goal_(0)) < bound_x &&
       std::abs(start_position(1) - goal_(1)) < bound_y &&
       std::abs(start_position(2) - goal_(2)) < bound_z))
    reached_goal_ = true;

  if (reached_goal_) {
    ROS_INFO("%s: Reached end of trajectory. Hovering in place.", name_.c_str());

    // Same point == goal, but three times.
    const std::vector<Vector3d> positions = { goal_, goal_, goal_ };

    // Get the bound value.
    const ValueFunctionId bound_value = (traj_ == nullptr) ?
      planners_.front()->GetIncomingValueFunction() :
      traj_->GetControlValueFunction(current_time.toSec());

    // Get control value.
    const ValueFunctionId control_value =
      planners_.back()->GetOutgoingValueFunction();

    // Make sure switching time server is up.
    if (!switching_time_srv_) {
      ROS_WARN("%s: Switching time server disconnected.", name_.c_str());

      ros::NodeHandle nl;
      switching_time_srv_ = nl.serviceClient<value_function::GuaranteedSwitchingTime>(
        switching_time_name_.c_str(), true);
      return;
    }

    // Get times.
    double switching_time = 10.0;
    value_function::GuaranteedSwitchingTime t;
    t.request.from_id = bound_value;
    t.request.to_id = control_value;
    if (!switching_time_srv_.call(t))
      ROS_ERROR("%s: Error calling switching time server.", name_.c_str());
    else
      switching_time = std::max(std::max(t.response.x, t.response.y),
                                t.response.z);

    const std::vector<double> times =
      { current_time.toSec(),
        current_time.toSec() + switching_time + 0.1,
        current_time.toSec() + switching_time + 100.0 };

    // Set up values.
    const std::vector<ValueFunctionId> bound_values =
      { bound_value, control_value, control_value };
    const std::vector<ValueFunctionId> control_values =
      { control_value, control_value, control_value };
    const std::vector<VectorXd> states =
      dynamics_->LiftGeometricTrajectory(positions, times);

    // Construct trajectory and publish.
    const Trajectory::Ptr hover =
      Trajectory::Create(times, states, control_values, bound_values);
    traj_ = hover;

    traj_pub_.publish(hover->ToRosMessage());
    return;
  }

  if (!Plan(start_position, goal_, start_time)) {
    ROS_ERROR("%s: MetaPlanner failed. Please come again.", name_.c_str());
    return;
  }

  ROS_INFO("%s: MetaPlanner succeeded after %2.5f seconds.",
           name_.c_str(), (ros::Time::now() - current_time).toSec());
}

// Plan a trajectory using the given (ordered) list of Planners.
// (1) Set up a new RRT-like structure to hold the meta plan.
// (2) Sample a new point in the state space.
// (3) Find nearest neighbor.
// (4) Plan a trajectory (starting with most aggressive planner).
// (5) Try to connect to the goal point.
// (6) Stop when we have a feasible trajectory. Otherwise go to (2).
// (7) When finished, convert to a message and publish.
bool MetaPlanner::Plan(const Vector3d& start, const Vector3d& stop,
                       double start_time) {
  // Only plan if position has been updated.
  //  if (!been_updated_)
  //    return false;

  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time current_time = ros::Time::now();
  const ValueFunctionId start_value = (traj_ == nullptr) ?
    planners_.back()->GetOutgoingValueFunction() :
    traj_->GetBoundValueFunction(start_time);

  WaypointTree tree(start, start_value, start_time);

  Vector3d sample(stop);

  // (3) Find the nearest neighbor.
  const size_t kNumNeighbors = 1;
  const std::vector<Waypoint::ConstPtr> neighbors =
    tree.KnnSearch(sample, kNumNeighbors);
  
  // Throw out this sample if too far from the nearest point.
  if (neighbors.size() != kNumNeighbors ||
      (neighbors[0]->point_ - sample).norm() > max_connection_radius_) {
    return false;
  }

  Waypoint::ConstPtr neighbor = neighbors[0];

  // Extract value function and corresponding planner ID from last waypoint.
  // If value is null, (i.e. at root) then set to planners_.size() since
  // any planner is valid from the root. Convert value ID to planner ID
  // by dividing by 2 since each planner has two value functions.
  const Trajectory::ConstPtr neighbor_traj = neighbor->traj_;
  const ValueFunctionId neighbor_val = neighbor->value_;

  const size_t neighbor_planner_id = neighbor_val / 2;

  // (4) Plan a trajectory using the most aggressive planner.
  const Planner::ConstPtr planner = planners_[0];

  const ValueFunctionId value_used = planner->GetIncomingValueFunction();
  const ValueFunctionId possible_next_value =
    planner->GetOutgoingValueFunction();

  const double time = (neighbor_traj == nullptr) ?
    start_time : neighbor_traj->LastTime();

  // Plan using 100% of the available total runtime.
  // NOTE! This is just a heuristic and could easily be changed.
  const Trajectory::Ptr traj = 
    planner->Plan(neighbor->point_, sample, time, max_runtime_);

  // Check if we found a trajectory to this sample.
  if (traj == nullptr) {
    ROS_WARN_THROTTLE(1.0, "A* failed!");
    return false;
  }

  // Insert the sample.
  const Waypoint::ConstPtr waypoint = Waypoint::Create(
    sample, value_used, traj, neighbor);

  tree.Insert(waypoint, true);

  // Chillax until we need to be done.
  const double time_remaining = (ros::Time::now() - current_time).toSec();
  if (time_remaining > 1e-4) {
    ros::Duration(time_remaining).sleep();
  }

  // If this was the first time through the loop then the sample was the
  // goal point, and we're done. Make sure to sleep until we would have
  // finished otherwise.
  ROS_INFO("A* succeeded after %f seconds.", 
	   (ros::Time::now() - current_time).toSec());

  // Get the best (fastest) trajectory out of the tree.
  const Trajectory::ConstPtr best = tree.BestTrajectory();
  ROS_INFO("%s: Publishing trajectory of length %zu.",
	   name_.c_str(), best->Size());

  traj_ = best;
  traj_pub_.publish(best->ToRosMessage());
  return true;
}

} //\namespace meta
