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
//    control: u = [phi, theta, w, T]
//
// Note: angle sign convention is based on positive accelerations on FLU frame
// (theta PITCH DOWN / phi ROLL LEFT / psi YAW left)

#ifndef VALUE_FUNCTION_NEAR_HOVER_DYNAMICS_H
#define VALUE_FUNCTION_NEAR_HOVER_DYNAMICS_H

#include <value_function/dynamics.h>
#include <utils/types.h>

namespace meta {

class NearHoverDynamics : public Dynamics {
public:
  typedef std::shared_ptr<const NearHoverDynamics> ConstPtr;

  // Destructor.
  ~NearHoverDynamics() {}

  // Factory method. Use this instead of the constructor.
  static NearHoverDynamics::ConstPtr Create(const VectorXd& lower_u,
                                            const VectorXd& upper_u);

  // Derived from parent virtual operator, gives the time derivative of state
  // as a function of current state and control. See above description for details.
  inline VectorXd Evaluate(const VectorXd& x, const VectorXd& u) const {
    VectorXd x_dot(X_DIM);
    x_dot(0) = x(3);
    x_dot(1) = x(4);
    x_dot(2) = x(5);
    x_dot(3) = u(3)*std::sin(u(1))*std::cos(x(6)) - u(3)*std::sin(u(0))*std::sin(x(6));
    x_dot(4) = u(3)*std::sin(u(0))*std::cos(x(6)) + u(3)*std::sin(u(1))*std::sin(x(6));
    x_dot(5) = u(3)*std::cos(u(0))*std::cos(u(1)) - constants::G;
    x_dot(6) = u(2);
    return x_dot;
  }

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the specified state.
  // This function is currently not implemented for NearHover.
  VectorXd OptimalControl(const VectorXd& x,
                          const VectorXd& value_gradient) const;

  // Puncture a full state vector and return a position.
  Vector3d Puncture(const VectorXd& x) const;

  // Get the corresponding full state dimension to the given spatial dimension.
  size_t SpatialDimension(size_t dimension) const;

  // Get the max acceleration in the given spatial dimension.
  double MaxAcceleration(size_t dimension) const;

  // Derived classes must be able to translate a geometric trajectory
  // (i.e. through Euclidean space) into a full state space trajectory.
  std::vector<VectorXd> LiftGeometricTrajectory(
    const std::vector<Vector3d>& positions,
    const std::vector<double>& times) const;

private:
  // Private constructor. Use the factory method instead.
  explicit NearHoverDynamics(const VectorXd& lower_u,
                             const VectorXd& upper_u);

  // Static dimensions.
  static const size_t X_DIM;
  static const size_t U_DIM;
};

} //\namespace meta

#endif
