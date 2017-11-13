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
// Defines the LinearDynamics class.
//
///////////////////////////////////////////////////////////////////////////////

#include <value_function/linear_dynamics.h>

namespace meta {

// Factory method. Use this instead of the constructor.
LinearDynamics::ConstPtr LinearDynamics::Create(const MatrixXd& A,
                                                const MatrixXd& B,
                                                const VectorXd& lower_u,
                                                const VectorXd& upper_u) {
  LinearDynamics::ConstPtr ptr(new LinearDynamics(A, B, lower_u, upper_u));
  return ptr;
}

// Derived classes must be able to compute an optimal control given
// the gradient of the value function at the specified state.
// In this case (linear dynamics), the state is irrelevant given the
// gradient of the value function at that state.
VectorXd LinearDynamics::OptimalControl(const VectorXd& x,
                                        const VectorXd& value_gradient) const {
  // Project the value gradient onto the dynamics.
  const VectorXd proj_gradient = B_.transpose() * value_gradient;

  // Set each dimension of optimal control to upper/lower bound depending
  // on the sign of the gradient in that dimension. We want to minimize the
  // inner product between the projected gradient and control.
  // If the gradient is 0, then sets control to zero by default.
  VectorXd optimal_control(VectorXd::Zero(lower_u_.size()));

  for (size_t ii = 0; ii < lower_u_.size(); ii++) {
    if (proj_gradient(ii) < 0.0)
      optimal_control(ii) = upper_u_(ii);
    else if (proj_gradient(ii) > 0.0)
      optimal_control(ii) = lower_u_(ii);
  }

  return optimal_control;
}

// Puncture a full state vector and return a position.
Vector3d LinearDynamics::Puncture(const VectorXd& x) const {
  ROS_WARN("Puncture is unimplemented.");
  return Vector3d::Zero();
}

// Get the corresponding full state dimension to the given spatial dimension.
size_t LinearDynamics::SpatialDimension(size_t dimension) const {
  ROS_WARN("SpatialDimension is unimplemented.");

  return dimension;
}

// Get the max acceleration in the given spatial dimension.
double LinearDynamics::MaxAcceleration(size_t dimension) const {
  ROS_WARN("MaxAcceleration is unimplemented.");

  return 0.0;
}

// Derived classes must be able to translate a geometric trajectory
// (i.e. through Euclidean space) into a full state space trajectory.
std::vector<VectorXd> LinearDynamics::LiftGeometricTrajectory(
  const std::vector<Vector3d>& positions,
  const std::vector<double>& times) const {
  ROS_WARN("LiftGeometricTrajectory is unimplemented.");

  std::vector<VectorXd> states;
  for (const auto& p : positions) {
    VectorXd state(3);
    state(0) = p(0);
    state(1) = p(1);
    state(2) = p(2);
    states.push_back(state);
  }

  return states;
}

// Private constructor. Use the factory method instead.
LinearDynamics::LinearDynamics(const MatrixXd& A, const MatrixXd& B,
                               const VectorXd& lower_u, const VectorXd& upper_u)
  : Dynamics(lower_u, upper_u),
    A_(A),
    B_(B) {
  // Check that dimensions match.
  if (A_.rows() != A_.cols())
    ROS_ERROR("A matrix is not square.");

  if (A_.rows() != B_.rows())
    ROS_ERROR("A and B matrices have different numbers of rows.");

  if (B_.cols() != lower_u_.size())
    ROS_ERROR("B matrix width does not match control dimension.");
}

} //\namespace meta
