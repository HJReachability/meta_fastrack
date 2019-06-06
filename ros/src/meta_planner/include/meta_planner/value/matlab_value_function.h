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

#ifndef META_PLANNER_VALUE_MATLAB_VALUE_FUNCTION_H
#define META_PLANNER_VALUE_MATLAB_VALUE_FUNCTION_H

#include <fastrack/dynamics/dynamics.h>
#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/relative_state.h>
#include <fastrack/utils/matlab_file_reader.h>
#include <fastrack/utils/types.h>
#include <fastrack/bound/box.h>

#include <fastrack_msgs/Control.h>
#include <fastrack_msgs/State.h>

#include <ros/assert.h>
#include <ros/ros.h>
#include <functional>

namespace meta_planner {
namespace value {

class MatlabValueFunction {
 public:
  ~MatlabValueFunction() {}
  MatlabValueFunction() : name_("value_function"), initialized_(false) {}

  // Initialize from file. Returns whether or not loading was successful.
  bool InitializeFromMatFile(const std::string& file_name);

  // Value and gradient at particular relative states.
  double Value(const VectorXd& relative_x) const;
  VectorXd Gradient(const VectorXd& relative_x) const;

  // Priority of the optimal control at the given tracker and planner states.
  // This is a number between 0 and 1, where 1 means the final control signal
  // should be exactly the optimal control signal computed by this
  // value function.
  double Priority(const VectorXd& relative_x) const;

  // Get the optimal control given the tracker state and planner state.
  fastrack_msgs::Control OptimalControl(
      const fastrack_msgs::State& tracker_x_msg,
      const fastrack_msgs::State& planner_x_msg) const;

  // Accessors.
  const fastrack::bound::Box& TrackingBound() const {
    return bound_;
  }
  const std::vector<double>& TrackingBoundParams() const {
    return bound_params_;
  }
  const std::vector<double>& TrackerDynamicsParams() const {
    return tracker_dynamics_params_;
  }
  const std::vector<double>& PlannerDynamicsParams() const {
    return planner_dynamics_params_;
  }

 private:
  // Convert a (relative) state to an index into 'data_'.
  size_t StateToIndex(const VectorXd& x) const;

  // Compute the difference vector between this (relative) state and the center
  // of the nearest cell (i.e. cell center minus state).
  VectorXd DirectionToCenter(const VectorXd& x) const;

  // Accessor for precomputed gradient at the given state.
  VectorXd GradientAccessor(const VectorXd& x) const;

  // Compute the grid point below a given state in dimension idx.
  double LowerGridPoint(const VectorXd& x, size_t idx) const;

  // Compute center of nearest grid cell to the given state.
  VectorXd NearestCenterPoint(const VectorXd& x) const;

  // Recursive helper function for gradient multilinear interpolation.
  // Takes in a state and index along which to interpolate.
  VectorXd RecursiveGradientInterpolator(const VectorXd& x, size_t idx) const;

  // Lower and upper bounds for the value function. Used for computing the
  // 'priority' of the optimal control signal.
  double priority_lower_;
  double priority_upper_;

  // Number of cells and upper/lower bounds in each dimension.
  std::vector<size_t> num_cells_;
  std::vector<double> cell_size_;
  std::vector<double> lower_;
  std::vector<double> upper_;

  // Value function itself is stored in row-major order.
  std::vector<double> data_;

  // Gradient information at each cell. One list per dimension, each in the
  // same order as 'data_'.
  std::vector<std::vector<double>> gradient_;

  // Dynamics parameters (to be read from mat file).
  std::vector<double> tracker_dynamics_params_;
  std::vector<double> planner_dynamics_params_;

  // Names of state types.
  std::string tracker_state_type_;
  std::string planner_state_type_;
  bool is_planner_kinematic_;

  // Keep a copy of the tracking error bound.
  fastrack::bound::Box bound_;
  std::vector<double> bound_params_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};  //\class MatlabValueFunction

}  // namespace value
}  // namespace meta_planner

#endif
