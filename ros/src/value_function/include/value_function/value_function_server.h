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

#ifndef VALUE_FUNCTION_VALUE_FUNCTION_SERVER_H
#define VALUE_FUNCTION_VALUE_FUNCTION_SERVER_H

#include <value_function/value_function.h>
#include <value_function/analytical_point_mass_value_function.h>
#include <value_function/dynamics.h>
#include <value_function/near_hover_quad_no_yaw.h>
#include <utils/types.h>
#include <utils/uncopyable.h>

#include <value_function/Priority.h>
#include <value_function/OptimalControl.h>
#include <value_function/GeometricPlannerSpeed.h>
#include <value_function/GeometricPlannerTime.h>
#include <value_function/GuaranteedSwitchingDistance.h>
#include <value_function/GuaranteedSwitchingTime.h>
#include <value_function/TrackingBoundBox.h>
#include <value_function/SwitchingTrackingBoundBox.h>

#include <ros/ros.h>

namespace meta {

class ValueFunctionServer : private Uncopyable {
public:
  ~ValueFunctionServer() {}
  explicit ValueFunctionServer()
    : initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

  // Get the optimal control at a particular state.
  bool OptimalControlCallback(
    value_function::OptimalControl::Request& req,
    value_function::OptimalControl::Response& res);

  // Get the tracking error bound in this spatial dimension.
  bool TrackingBoundCallback(
    value_function::TrackingBoundBox::Request& req,
    value_function::TrackingBoundBox::Response& res);

  // Get the tracking error bound in this spatial dimension for a planner
  // switching INTO this one with the specified max speed.
  bool SwitchingTrackingBoundCallback(
    value_function::SwitchingTrackingBoundBox::Request& req,
    value_function::SwitchingTrackingBoundBox::Response& res);

  // Guaranteed time in which a planner with the specified value function
  // can switch into this value function's tracking error bound.
  bool GuaranteedSwitchingTimeCallback(
    value_function::GuaranteedSwitchingTime::Request& req,
    value_function::GuaranteedSwitchingTime::Response& res);

  // Guaranteed distance in which a planner with the specified value function
  // can switch into this value function's safe set.
  bool GuaranteedSwitchingDistanceCallback(
    value_function::GuaranteedSwitchingDistance::Request& req,
    value_function::GuaranteedSwitchingDistance::Response& res);

  // Priority of the optimal control at the given state. This is a number
  // between 0 and 1, where 1 means the final control signal should be exactly
  // the optimal control signal computed by this value function.
  bool PriorityCallback(value_function::Priority::Request& req,
                        value_function::Priority::Response& res);

  // Max planner speed in the given spatial dimension.
  bool MaxPlannerSpeedCallback(
    value_function::GeometricPlannerSpeed::Request& req,
    value_function::GeometricPlannerSpeed::Response& res);

  // Compute the shortest possible time to go from start to stop for a
  // geometric planner with the max planner speed for this value function.
  bool BestPossibleTimeCallback(
    value_function::GeometricPlannerTime::Request& req,
    value_function::GeometricPlannerTime::Response& res);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Services.
  ros::ServiceServer optimal_control_srv_;
  ros::ServiceServer tracking_bound_srv_;
  ros::ServiceServer switching_tracking_bound_srv_;
  ros::ServiceServer guaranteed_switching_time_srv_;
  ros::ServiceServer guaranteed_switching_distance_srv_;
  ros::ServiceServer priority_srv_;
  ros::ServiceServer max_planner_speed_srv_;
  ros::ServiceServer best_possible_time_srv_;

  std::string optimal_control_name_;
  std::string tracking_bound_name_;
  std::string switching_tracking_bound_name_;
  std::string guaranteed_switching_time_name_;
  std::string guaranteed_switching_distance_name_;
  std::string priority_name_;
  std::string max_planner_speed_name_;
  std::string best_possible_time_name_;

  // Numerical mode flag and associated parameters for both analytic
  // and numerical modes.
  bool numerical_mode_;
  std::vector<std::string> value_dirs_;
  std::vector<double> max_planner_speeds_;
  std::vector<double> max_velocity_disturbances_;
  std::vector<double> max_acceleration_disturbances_;

  // Control upper/lower bounds.
  size_t control_dim_, state_dim_;
  std::vector<double> control_upper_;
  std::vector<double> control_lower_;

  // List of value functions.
  std::vector<ValueFunction::ConstPtr> values_;

  // Initialization and naming.
  bool initialized_;
  std::string name_;
};

} //\namespace meta

#endif
