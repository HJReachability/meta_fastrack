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
// Defines the ValueFunctionServer class, which manages the service-based
// interface to all value functions.
//
///////////////////////////////////////////////////////////////////////////////

#include <value_function/value_function_server.h>
#include <utils/message_interfacing.h>

namespace meta {

// Initialize this class with all parameters and callbacks.
bool ValueFunctionServer::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "value_function_server");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Convert control bounds to Eigen format.
  VectorXd control_upper_vec(control_dim_);
  VectorXd control_lower_vec(control_dim_);
  for (size_t ii = 0; ii < control_dim_; ii++) {
    control_upper_vec(ii) = control_upper_[ii];
    control_lower_vec(ii) = control_lower_[ii];
  }

  // Set up dynamics.
  NearHoverQuadNoYaw::ConstPtr dynamics =
    NearHoverQuadNoYaw::Create(control_lower_vec, control_upper_vec);

  // Create value functions.
  if (numerical_mode_) {
    for (size_t ii = 0; ii < value_dirs_.size(); ii++) {
      const ValueFunction::ConstPtr value =
        ValueFunction::Create(value_dirs_[ii], dynamics,
                              state_dim_, control_dim_,
                              static_cast<ValueFunctionId>(ii));

      values_.push_back(value);
    }
  } else {
    for (size_t ii = 0; ii < max_planner_speeds_.size(); ii++) {
      // Generate inputs for AnalyticalPointMassValueFunction.
      // SEMI-HACK! Manually feeding control/disturbance bounds.
      const Vector3d max_planner_speed =
        Vector3d::Constant(max_planner_speeds_[ii]);
      const Vector3d max_velocity_disturbance =
        Vector3d::Constant(max_velocity_disturbances_[ii]);
      const Vector3d max_acceleration_disturbance =
        Vector3d::Constant(max_acceleration_disturbances_[ii]);
      const Vector3d velocity_expansion = Vector3d::Constant(0.1);

      // Create analytical value function.
      const AnalyticalPointMassValueFunction::ConstPtr value =
        AnalyticalPointMassValueFunction::Create(max_planner_speed,
                                                 max_velocity_disturbance,
                                                 max_acceleration_disturbance,
                                                 velocity_expansion,
                                                 dynamics,
                                                 static_cast<ValueFunctionId>(ii));

      values_.push_back(value);
    }
  }

  // Make sure value functions were provided in pairs.
  if (values_.size() % 2 != 0) {
    ROS_ERROR("%s: Must provide value functions in pairs.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Get the optimal control at a particular state.
bool ValueFunctionServer::OptimalControlCallback(
  value_function_srvs::OptimalControl::Request& req,
  value_function_srvs::OptimalControl::Response& res) {
  const VectorXd state = utils::Unpack(req.state);
  const VectorXd control = values_[req.id]->OptimalControl(state);
  res.control = utils::PackControl(control);

  return true;
}

// Get the tracking error bound in this spatial dimension.
bool ValueFunctionServer::TrackingBoundCallback(
  value_function_srvs::TrackingBoundBox::Request& req,
  value_function_srvs::TrackingBoundBox::Response& res) {
  res.x = values_[req.id]->TrackingBound(0);
  res.y = values_[req.id]->TrackingBound(1);
  res.z = values_[req.id]->TrackingBound(2);

  return true;
}

// Get the tracking error bound in this spatial dimension for a planner
// switching INTO this one with the specified max speed.
bool ValueFunctionServer::SwitchingTrackingBoundCallback(
  value_function_srvs::SwitchingTrackingBoundBox::Request& req,
  value_function_srvs::SwitchingTrackingBoundBox::Response& res) {
  // Check which mode we're in.
  if (numerical_mode_) {
    res.x = values_[req.to_id]->SwitchingTrackingBound(0, values_[req.from_id]);
    res.y = values_[req.to_id]->SwitchingTrackingBound(1, values_[req.from_id]);
    res.z = values_[req.to_id]->SwitchingTrackingBound(2, values_[req.from_id]);
  } else {
    const auto cast_to = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.to_id]);
    const auto cast_from = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.from_id]);

    res.x = cast_to->SwitchingTrackingBound(0, cast_from);
    res.y = cast_to->SwitchingTrackingBound(1, cast_from);
    res.z = cast_to->SwitchingTrackingBound(2, cast_from);
  }

  return true;
}

// Guaranteed time in which a planner with the specified value function
// can switch into this value function's tracking error bound.
bool ValueFunctionServer::GuaranteedSwitchingTimeCallback(
  value_function_srvs::GuaranteedSwitchingTime::Request& req,
  value_function_srvs::GuaranteedSwitchingTime::Response& res) {
  // Check which mode we're in.
  if (numerical_mode_) {
    res.x = values_[req.to_id]->GuaranteedSwitchingTime(0, values_[req.from_id]);
    res.y = values_[req.to_id]->GuaranteedSwitchingTime(1, values_[req.from_id]);
    res.z = values_[req.to_id]->GuaranteedSwitchingTime(2, values_[req.from_id]);
  } else {
    const auto cast_to = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.to_id]);
    const auto cast_from = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.from_id]);

    res.x = cast_to->GuaranteedSwitchingTime(0, cast_from);
    res.y = cast_to->GuaranteedSwitchingTime(1, cast_from);
    res.z = cast_to->GuaranteedSwitchingTime(2, cast_from);
  }

  return true;
}

// Guaranteed distance in which a planner with the specified value function
// can switch into this value function's safe set.
bool ValueFunctionServer::GuaranteedSwitchingDistanceCallback(
  value_function_srvs::GuaranteedSwitchingDistance::Request& req,
  value_function_srvs::GuaranteedSwitchingDistance::Response& res) {
  // Check which mode we're in.
  if (numerical_mode_) {
    res.x = values_[req.to_id]->GuaranteedSwitchingDistance(0, values_[req.from_id]);
    res.y = values_[req.to_id]->GuaranteedSwitchingDistance(1, values_[req.from_id]);
    res.z = values_[req.to_id]->GuaranteedSwitchingDistance(2, values_[req.from_id]);
  } else {
    const auto cast_to = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.to_id]);
    const auto cast_from = std::static_pointer_cast<
      const AnalyticalPointMassValueFunction>(values_[req.from_id]);

    res.x = cast_to->GuaranteedSwitchingDistance(0, cast_from);
    res.y = cast_to->GuaranteedSwitchingDistance(1, cast_from);
    res.z = cast_to->GuaranteedSwitchingDistance(2, cast_from);
  }

  return true;
}

// Priority of the optimal control at the given state. This is a number
// between 0 and 1, where 1 means the final control signal should be exactly
// the optimal control signal computed by this value function.
bool ValueFunctionServer::PriorityCallback(
  value_function_srvs::Priority::Request& req,
  value_function_srvs::Priority::Response& res) {
  const VectorXd state = utils::Unpack(req.state);
  res.priority = values_[req.id]->Priority(state);
  return true;
}

// Max planner speed in the given spatial dimension.
bool ValueFunctionServer::MaxPlannerSpeedCallback(
  value_function_srvs::GeometricPlannerSpeed::Request& req,
  value_function_srvs::GeometricPlannerSpeed::Response& res) {
  res.x = values_[req.id]->MaxPlannerSpeed(0);
  res.y = values_[req.id]->MaxPlannerSpeed(1);
  res.z = values_[req.id]->MaxPlannerSpeed(2);
  return true;
}

// Compute the shortest possible time to go from start to stop for a
// geometric planner with the max planner speed for this value function.
bool ValueFunctionServer::BestPossibleTimeCallback(
  value_function_srvs::GeometricPlannerTime::Request& req,
  value_function_srvs::GeometricPlannerTime::Response& res) {
  const Vector3d start = utils::Unpack(req.start);
  const Vector3d stop = utils::Unpack(req.stop);
  res.time = values_[req.id]->BestPossibleTime(start, stop);

  return true;
}

// Load parameters.
bool ValueFunctionServer::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Numerical mode flag and associated parameters for loading value functions.
  if (!nl.getParam("numerical_mode", numerical_mode_)) return false;
  if (!nl.getParam("planners/value_directories", value_dirs_)) return false;

  if (value_dirs_.size() == 0) {
    ROS_ERROR("%s: Must specify at least one value function directory.",
              name_.c_str());
    return false;
  }

  if (!nl.getParam("planners/max_speeds", max_planner_speeds_)) return false;
  if (!nl.getParam("planners/max_velocity_disturbances",
                   max_velocity_disturbances_)) return false;
  if (!nl.getParam("planners/max_acceleration_disturbances",
                   max_acceleration_disturbances_)) return false;

  if (max_planner_speeds_.size() != max_velocity_disturbances_.size() ||
      max_planner_speeds_.size() != max_acceleration_disturbances_.size()) {
    ROS_ERROR("%s: Must specify max speed/velocity/acceleration disturbances.",
              name_.c_str());
    return false;
  }

  // Dimensions and control bounds.
  int dimension = 1;
  if (!nl.getParam("control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("control/upper", control_upper_)) return false;
  if (!nl.getParam("control/lower", control_lower_)) return false;

  if (control_upper_.size() != control_dim_ ||
      control_lower_.size() != control_dim_) {
    ROS_ERROR("%s: Upper and/or lower bounds are in the wrong dimension.",
              name_.c_str());
    return false;
  }

  // Names of all services.
  if (!nl.getParam("srv/optimal_control", optimal_control_name_)) return false;
  if (!nl.getParam("srv/tracking_bound", tracking_bound_name_)) return false;
  if (!nl.getParam("srv/switching_tracking_bound",
                   switching_tracking_bound_name_)) return false;
  if (!nl.getParam("srv/guaranteed_switching_time",
                   guaranteed_switching_time_name_)) return false;
  if (!nl.getParam("srv/guaranteed_switching_distance",
                   guaranteed_switching_distance_name_)) return false;
  if (!nl.getParam("srv/priority", priority_name_)) return false;
  if (!nl.getParam("srv/max_planner_speed",
                   max_planner_speed_name_)) return false;
  if (!nl.getParam("srv/best_possible_time",
                   best_possible_time_name_)) return false;

  return true;
}

// Set up all servers.
bool ValueFunctionServer::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  optimal_control_srv_ = nl.advertiseService(
    optimal_control_name_,
    &ValueFunctionServer::OptimalControlCallback, this);
  tracking_bound_srv_ = nl.advertiseService(
    tracking_bound_name_,
    &ValueFunctionServer::TrackingBoundCallback, this);
  switching_tracking_bound_srv_ = nl.advertiseService(
    switching_tracking_bound_name_,
    &ValueFunctionServer::SwitchingTrackingBoundCallback, this);
  guaranteed_switching_time_srv_ = nl.advertiseService(
    guaranteed_switching_time_name_,
    &ValueFunctionServer::GuaranteedSwitchingTimeCallback, this);
  guaranteed_switching_distance_srv_ = nl.advertiseService(
    guaranteed_switching_distance_name_,
    &ValueFunctionServer::GuaranteedSwitchingDistanceCallback, this);
  priority_srv_ = nl.advertiseService(
    priority_name_, &ValueFunctionServer::PriorityCallback, this);
  max_planner_speed_srv_ = nl.advertiseService(
    max_planner_speed_name_,
    &ValueFunctionServer::MaxPlannerSpeedCallback, this);
  best_possible_time_srv_ = nl.advertiseService(
    best_possible_time_name_,
    &ValueFunctionServer::BestPossibleTimeCallback, this);

  return true;
}

} //\namespace meta
