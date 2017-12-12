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
// Defines the NearHoverQuadNoYaw class. Assumes that state 'x' entried are:
// * x(0) -- x
// * x(1) -- y
// * x(2) -- z
// * x(3) -- x_dot
// * x(4) -- y_dot
// * x(5) -- z_dot
//
// Also assumes that entried in control 'u' are:
// * u(0) -- pitch
// * u(1) -- roll
// * u(2) -- thrust
//
///////////////////////////////////////////////////////////////////////////////

#include <value_function/near_hover_quad_no_yaw.h>

namespace meta {

// Dimensions.
const size_t NearHoverQuadNoYaw::X_DIM = 6;
const size_t NearHoverQuadNoYaw::U_DIM = 3;

// Factory method. Use this instead of the constructor.
NearHoverQuadNoYaw::ConstPtr NearHoverQuadNoYaw::
Create(const VectorXd& lower_u, const VectorXd& upper_u) {
  NearHoverQuadNoYaw::ConstPtr ptr(new NearHoverQuadNoYaw(lower_u, upper_u));
  return ptr;
}

// Derived classes must be able to compute an optimal control given
// the gradient of the value function at the specified state.
// In this case (linear dynamics), the state is irrelevant given the
// gradient of the value function at that state.
VectorXd NearHoverQuadNoYaw::OptimalControl(
  const VectorXd& x, const VectorXd& value_gradient) const {

  // Set each dimension of optimal control to upper/lower bound depending
  // on the sign of the gradient in that dimension. We want to minimize the
  // inner product between the projected gradient and control.
  // If the gradient is 0, then sets control to zero by default.
  VectorXd optimal_control(VectorXd::Zero(lower_u_.size()));
  optimal_control(0) = (value_gradient(3) < 0.0) ? upper_u_(0) : lower_u_(0);
  optimal_control(1) = (value_gradient(4) > 0.0) ? upper_u_(1) : lower_u_(1);
  optimal_control(2) = (value_gradient(5) < 0.0) ? upper_u_(2) : lower_u_(2);

  return optimal_control;
}

// Get the corresponding full state dimension to the given spatial dimension.
size_t NearHoverQuadNoYaw::SpatialDimension(size_t dimension) const {
  if (dimension == 0)
    return 0;
  if (dimension == 1)
    return 1;
  if (dimension == 2)
    return 2;

  ROS_ERROR("Invalid spatial dimension.");
  return 0;
}

// Puncture a full state vector and return a position.
Vector3d NearHoverQuadNoYaw::Puncture(const VectorXd& x) const {
  return Vector3d(x(0), x(1), x(2));
}

// Get the max acceleration in the given spatial dimension.
double NearHoverQuadNoYaw::MaxAcceleration(size_t dimension) const {
  if (dimension == 0)
    return constants::G * std::max(std::abs(std::tan(upper_u_(0))),
                                   std::abs(std::tan(lower_u_(0))));
  if (dimension == 1)
    return constants::G * std::max(std::abs(std::tan(upper_u_(1))),
                                   std::abs(std::tan(lower_u_(1))));
  if (dimension == 2)
    return std::max(std::abs(upper_u_(2) - constants::G),
                    std::abs(lower_u_(2) - constants::G));

  ROS_ERROR("Invalid spatial dimension.");
  return 0.0;
}

// Derived classes must be able to translate a geometric trajectory
// (i.e. through Euclidean space) into a full state space trajectory.
std::vector<VectorXd> NearHoverQuadNoYaw::LiftGeometricTrajectory(
  const std::vector<Vector3d>& positions,
  const std::vector<double>& times) const {

  // Number of entries in trajectory.
  size_t num_waypoints = positions.size();

#ifdef ENABLE_DEBUG_MESSAGES
  if (positions.size() != times.size()) {
    ROS_WARN("NearHoverQuadNoYaw: Inconsistent number of states and times.");
    num_waypoints = std::min(positions.size(), times.size());
  }

  if (num_waypoints == 0) {
    ROS_WARN("NearHoverQuadNoYaw: No waypoints provided.");
    return std::vector<VectorXd>();
  }
#endif

  // Create a clean, empty trajectory.
  std::vector<VectorXd> full_states;
  Vector3d velocity(Vector3d::Zero());

  // Loop through the geometric trajectory and get the velocity with a
  // forward difference.
  for (size_t ii = 0; ii < num_waypoints - 1; ii++) {
    velocity = (positions[ii + 1] - positions[ii]) / (times[ii + 1] - times[ii]);

    // Populate full state vector.
    VectorXd full(X_DIM);
    full(0) = positions[ii](0);
    full(1) = positions[ii](1);
    full(2) = positions[ii](2);

    full(3) = velocity(0);
    full(4) = velocity(1);
    full(5) = velocity(2);

    // Append to trajectory.
    full_states.push_back(full);
  }

  // Catch final waypoint.
  VectorXd full(X_DIM);
  full(0) = positions.back()(0);
  full(1) = positions.back()(1);
  full(2) = positions.back()(2);

  full(3) = velocity(0);
  full(4) = velocity(1);
  full(5) = velocity(2);

  full_states.push_back(full);

  return full_states;
}

// Private constructor. Use the factory method instead.
NearHoverQuadNoYaw::NearHoverQuadNoYaw(
  const VectorXd& lower_u, const VectorXd& upper_u)
  : Dynamics(lower_u, upper_u) {}

} //\namespace meta
