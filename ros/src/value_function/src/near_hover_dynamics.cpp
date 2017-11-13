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
 * Authors: Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
 *          David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the NearHoverDynamics class (7D quadrotor model used by Crazyflie).
//
///////////////////////////////////////////////////////////////////////////////

// System dynamics
// (quadrotor in near hover, assuming small attitude angles)
//    dx/dt     =  vx_G
//    dy/dt     =  vy_G
//    dz/dt     =  vz_G
//    dvx_G/dt  =  T*sin(theta)*cos(psi) - T*sin(phi)*sin(psi)
//    dvy_G/dt  =  T*sin(phi)*cos(psi)   + T*sin(theta)*sin(psi)
//    dvz_G/dt  =  T*cos(phi)*cos(theta) - g
//    dpsi/dt   =  w
//
//    control: u = [ phi, theta, w, T]
//
// Note: angle sign convention is based on positive accelerations on FLU frame
// (theta PITCH DOWN / phi ROLL LEFT / psi YAW left)

#include <value_function/near_hover_dynamics.h>

namespace meta {

const size_t NearHoverDynamics::X_DIM = 7;
const size_t NearHoverDynamics::U_DIM = 4;

// Factory method. Use this instead of the constructor.
NearHoverDynamics::ConstPtr NearHoverDynamics::Create(const VectorXd& lower_u,
                                                      const VectorXd& upper_u) {
  NearHoverDynamics::ConstPtr ptr(new NearHoverDynamics(lower_u, upper_u));
  return ptr;
}

// Derived classes must be able to compute an optimal control given
// the gradient of the value function at the specified state.
// This function is currently not implemented for NearHover.
VectorXd NearHoverDynamics::OptimalControl(const VectorXd& x,
                                           const VectorXd& value_gradient) const {

  // The optimal control is the solution to a nonconvex optimization problem
  // with the inner product <grad(V),xdot(x,u)> as the objective.
  // No solution method is currently implemented.
  // Instead, the optimal control is set to zero by default.
  // TODO!
  ROS_WARN("Unimplemented method NearHoverDynamics::OptimalControl.");
  VectorXd optimal_control(VectorXd::Zero(lower_u_.size()));

  return optimal_control;
}

// Private constructor. Use the factory method instead.
NearHoverDynamics::NearHoverDynamics(const VectorXd& lower_u,
                                     const VectorXd& upper_u)
  : Dynamics(lower_u, upper_u) {}

// Puncture a full state vector and return a position.
Vector3d NearHoverDynamics::Puncture(const VectorXd& x) const {
  return Vector3d(x(0), x(1), x(2));
}

// Get the corresponding full state dimension to the given spatial dimension.
size_t NearHoverDynamics::SpatialDimension(size_t dimension) const {
  if (dimension == 0)
    return 0;
  if (dimension == 1)
    return 1;
  if (dimension == 2)
    return 2;

  ROS_ERROR("Invalid spatial dimension.");
  return 0;
}

// Get the max acceleration in the given spatial dimension.
double NearHoverDynamics::MaxAcceleration(size_t dimension) const {
  ROS_WARN("Unimplemented method MaxAcceleration.");

  return 0.0;
}


// Derived classes must be able to translate a geometric trajectory
// (i.e. through Euclidean space) into a full state space trajectory.
std::vector<VectorXd> NearHoverDynamics::
LiftGeometricTrajectory(const std::vector<Vector3d>& positions,
                        const std::vector<double>& times) const {
  // Number of entries in trajectory.
  size_t num_waypoints = positions.size();

#ifdef ENABLE_DEBUG_MESSAGES
  if (positions.size() != times.size()) {
    ROS_WARN("Inconsistent number of states and times.");
    num_waypoints = std::min(positions.size(), times.size());
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

    // Assume zero yaw.
    full(6) = 0.0;

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

  // Assume zero yaw.
  full(6) = 0.0;

  full_states.push_back(full);

  return full_states;
}

} //\namespace meta
