/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
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
// Defines the MatlabValueFunction class. NOTE that this class is a standalone
// non-templated version of the fastrack::value::MatlabValueFunction class,
// modified to work for meta planning.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/kinematics.h>
#include <fastrack/dynamics/quadrotor_decoupled_6d.h>
#include <fastrack/dynamics/quadrotor_decoupled_6d_rel_kinematics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/position_velocity.h>
#include <fastrack/state/position_velocity_rel_position_velocity.h>
#include <fastrack/state/relative_state.h>
#include <fastrack/utils/matlab_file_reader.h>
#include <fastrack/utils/types.h>
#include <meta_planner/value/matlab_value_function.h>

#include <fastrack_msgs/State.h>

#include <ros/assert.h>
#include <ros/ros.h>
#include <functional>

namespace meta_planner {
namespace value {

// Value at the given relative state.
double MatlabValueFunction::Value(const VectorXd& relative_x) const {
  // Get distance from cell center in each dimension.
  const VectorXd center_distance = DirectionToCenter(relative_x);

  // Interpolate.
  const double nn_value = data_[StateToIndex(relative_x)];
  double approx_value = nn_value;

  VectorXd neighbor = relative_x;
  for (size_t ii = 0; ii < relative_x.size(); ii++) {
    // Get neighboring value.
    if (center_distance(ii) >= 0.0)
      neighbor(ii) += cell_size_[ii];
    else
      neighbor(ii) -= cell_size_[ii];

    const double neighbor_value = data_[StateToIndex(neighbor)];
    neighbor(ii) = relative_x(ii);

    // Compute forward difference.
    const double slope = (center_distance(ii) >= 0.0)
                             ? (neighbor_value - nn_value) / cell_size_[ii]
                             : (nn_value - neighbor_value) / cell_size_[ii];

    // Add to the Taylor approximation.
    approx_value += slope * center_distance(ii);
  }

  return approx_value;
}

// Gradient at the given relative state.
VectorXd MatlabValueFunction::Gradient(const VectorXd& relative_x) const {
  // std::cout << "tracker_x: " << tracker_x.ToVector().transpose() <<
  // std::endl;
  // std::cout << "planner_x: " << planner_x.ToVector().transpose() <<
  // std::endl;
  // std::cout << "relative_x: " << relative_x.transpose() << std::endl;
  // std::cout << "gradient: " << RecursiveGradientInterpolator(relative_x,
  // 0).transpose() << std::endl;

  return RecursiveGradientInterpolator(relative_x, 0);
}

// Get the optimal control given the tracker state and planner state.
fastrack_msgs::Control MatlabValueFunction::OptimalControl(
    const fastrack_msgs::State& tracker_x_msg,
    const fastrack_msgs::State& planner_x_msg) const {
  ROS_ASSERT(initialized_);

  if (tracker_state_type_ == "PositionVelocity" &&
      planner_state_type_ == "PositionVelocity" && is_planner_kinematic_) {
    // Construct tracker and planner dynamics from param lists.
    const fastrack::dynamics::QuadrotorDecoupled6D<
        fastrack::control::QuadrotorControlBoundBox>
        tracker_dynamics(tracker_dynamics_params_);
    const fastrack::dynamics::Kinematics<fastrack::state::PositionVelocity>
        planner_dynamics(planner_dynamics_params_);

    // Convert state msgs to raw types.
    const fastrack::state::PositionVelocity tracker_x(tracker_x_msg);
    const fastrack::state::PositionVelocity planner_x(planner_x_msg);
    const fastrack::state::PositionVelocityRelPositionVelocity relative_x(
        tracker_x, planner_x);

    // Compute value gradient and convert to relative state.
    const fastrack::state::PositionVelocityRelPositionVelocity grad(
        Gradient(relative_x.ToVector()));

    // Construct relative dynamics.
    const fastrack::dynamics::QuadrotorDecoupled6DRelKinematics<
        fastrack::control::QuadrotorControlBoundBox>
        relative_dynamics;

    // Compute optimal control and convert to msg output.
    constexpr double kPriority = 1.0;
    return relative_dynamics
        .OptimalControl(tracker_x, planner_x, grad,
                        tracker_dynamics.GetControlBound(),
                        planner_dynamics.GetControlBound())
        .ToRos(kPriority);
  }

  ROS_ERROR("%s: Bad state types.", name_.c_str());
  ROS_ASSERT(false);
  return fastrack_msgs::Control();
}

// Priority of the optimal control at the given tracker and planner states.
// This is a number between 0 and 1, where 1 means the final control signal
// should be exactly the optimal control signal computed by this
// value function.
double MatlabValueFunction::Priority(const VectorXd& relative_x) const {
  const double value = Value(relative_x);

  if (value < priority_lower_) return 0.0;

  // HACK! If value is too high, just use LQR instead.
  if (value > priority_upper_) return 0.0;
  return (value - priority_lower_) / (priority_upper_ - priority_lower_);
}

// Convert a (relative) state to an index into 'data_'.
size_t MatlabValueFunction::StateToIndex(const VectorXd& x) const {
  // Quantize each dimension of the state.
  std::vector<size_t> quantized;
  for (size_t ii = 0; ii < x.size(); ii++) {
    if (x(ii) < lower_[ii]) {
      ROS_WARN_THROTTLE(1.0,
                        "%s: State is too small in dimension %zu: %f vs %f",
                        this->name_.c_str(), ii, x(ii), lower_[ii]);
      quantized.push_back(0);
    } else if (x(ii) > upper_[ii]) {
      ROS_WARN_THROTTLE(1.0,
                        "%s: State is too large in dimension %zu: %f vs %f",
                        this->name_.c_str(), ii, x(ii), upper_[ii]);
      quantized.push_back(num_cells_[ii] - 1);
    } else {
      // In bounds, so quantize. This works because of 0-indexing and casting.
      quantized.push_back(
          static_cast<size_t>((x(ii) - lower_[ii]) / cell_size_[ii]));
    }
  }

  // Convert to row-major order.
  size_t idx = quantized[0];
  for (size_t ii = 1; ii < quantized.size(); ii++) {
    idx *= num_cells_[ii];
    idx += quantized[ii];
  }

  return idx;
}

// Accessor for precomputed gradient at the given state.
VectorXd MatlabValueFunction::GradientAccessor(const VectorXd& x) const {
  // Convert to index and read gradient one dimension at a time.
  const size_t idx = StateToIndex(x);

  VectorXd gradient(x.size());
  for (size_t ii = 0; ii < gradient.size(); ii++)
    gradient(ii) = gradient_[ii][idx];

  return gradient;
}

// Compute the difference vector between this (relative) state and the center
// of the nearest cell (i.e. cell center minus state).
VectorXd MatlabValueFunction::DirectionToCenter(const VectorXd& x) const {
  return NearestCenterPoint(x) - x;
}

// Compute the grid point below a given state in dimension idx.
double MatlabValueFunction::LowerGridPoint(const VectorXd& x,
                                           size_t idx) const {
  // Get center of nearest cell.
  const double center =
      0.5 * cell_size_[idx] + lower_[idx] +
      cell_size_[idx] * std::floor((x(idx) - lower_[idx]) / cell_size_[idx]);

  // Check if center is above us. If so, the lower bound is the cell below.
  return (center > x(idx)) ? center - cell_size_[idx] : center;
}

// Compute the center of the cell nearest to the given state.
VectorXd MatlabValueFunction::NearestCenterPoint(const VectorXd& x) const {
  VectorXd center(x.size());

  for (size_t ii = 0; ii < center.size(); ii++)
    center[ii] =
        0.5 * cell_size_[ii] + lower_[ii] +
        cell_size_[ii] * std::floor((x(ii) - lower_[ii]) / cell_size_[ii]);

  return center;
}

// Recursive helper function for gradient multilinear interpolation.
// Takes in a state and index along which to interpolate.
VectorXd MatlabValueFunction::RecursiveGradientInterpolator(const VectorXd& x,
                                                            size_t idx) const {
  // Assume x's entries prior to idx are equal to the upper/lower bounds of
  // the cell containing x.
  // Begin by computing the lower and upper bounds of the cell containing x
  // in dimension idx.
  const double lower = LowerGridPoint(x, idx);
  const double upper = lower + cell_size_[idx];

  // Compute the fractional distance between lower and upper.
  const double fractional_dist = (x(idx) - lower) / cell_size_[idx];

  // Split x along dimension idx.
  VectorXd x_lower = x;
  x_lower(idx) = lower;

  VectorXd x_upper = x;
  x_upper(idx) = upper;

  // Base case.
  if (idx == x.size() - 1) {
    return GradientAccessor(x_upper) * fractional_dist +
           GradientAccessor(x_lower) * (1.0 - fractional_dist);
  }

  // Recursive step.
  return RecursiveGradientInterpolator(x_upper, idx + 1) * fractional_dist +
         RecursiveGradientInterpolator(x_lower, idx + 1) *
             (1.0 - fractional_dist);
}

// Initialize from file. Returns whether or not loading was successful.
// Can be used as an alternative to intialization from a NodeHandle.
bool MatlabValueFunction::InitializeFromMatFile(const std::string& file_name) {
  // Open up this file.
  fastrack::MatlabFileReader reader(file_name);
  if (!reader.IsOpen()) return false;

  // Load each class variable.
  if (!reader.ReadScalar("priority_lower", &priority_lower_)) return false;
  if (!reader.ReadScalar("priority_upper", &priority_upper_)) return false;
  if (!reader.ReadVector("num_cells", &num_cells_)) return false;
  if (!reader.ReadVector("lower", &lower_)) return false;
  if (!reader.ReadVector("upper", &upper_)) return false;
  if (!reader.ReadVector("data", &data_)) return false;

  // Check loaded variables.
  if (priority_lower_ >= priority_upper_) {
    ROS_ERROR("%s: Priority lower bound above upper bound.",
              this->name_.c_str());
    return false;
  }

  if (num_cells_.size() != lower_.size() || lower_.size() != upper_.size()) {
    ROS_ERROR("%s: Dimensions do not match.", this->name_.c_str());
    return false;
  }

  const double total_num_cells = std::accumulate(
      num_cells_.begin(), num_cells_.end(), 1, std::multiplies<size_t>());
  if (total_num_cells == 0) {
    ROS_ERROR("%s: 0 total cells.", this->name_.c_str());
    return false;
  }

  if (data_.size() != total_num_cells) {
    ROS_ERROR("%s: Grid data was of the wrong size.", this->name_.c_str());
    return false;
  }

  // Compute cell size.
  for (size_t ii = 0; ii < num_cells_.size(); ii++)
    cell_size_.emplace_back((upper_[ii] - lower_[ii]) /
                            static_cast<double>(num_cells_[ii]));

  // Load gradients.
  for (size_t ii = 0; ii < num_cells_.size(); ii++) {
    gradient_.emplace_back();
    auto& partial = gradient_.back();
    if (!reader.ReadVector("deriv_" + std::to_string(ii), &partial)) {
      ROS_ERROR("%s: Could not read deriv_%zu.", this->name_.c_str(), ii);
      return false;
    }

    if (partial.size() != total_num_cells) {
      ROS_ERROR("%s: Partial derivative in dimension %zu had incorrect size.",
                this->name_.c_str(), ii);
      return false;
    }
  }

  // Load dynamics and bound parameters.
  if (!reader.ReadVector("tracker_params", &tracker_dynamics_params_))
    return false;
  if (!reader.ReadVector("planner_params", &planner_dynamics_params_))
    return false;

  size_t is_planner_kinematic;
  if (!reader.ReadScalar("is_planner_kinematic", &is_planner_kinematic))
    return false;
  is_planner_kinematic_ = static_cast<bool>(is_planner_kinematic);

  if (!reader.ReadVector("bound_params", &bound_params_)) return false;
  bound_.Initialize(bound_params_);

  // State types..
  if (!reader.ReadString("tracker_state_type", &tracker_state_type_))
    return false;

  if (!reader.ReadString("planner_state_type", &planner_state_type_))
    return false;


  initialized_ = true;
  return true;
}

}  // namespace value
}  // namespace meta_planner
