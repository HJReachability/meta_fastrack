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
// Defines the Dynamics class. All Dynamics will be constructed as ConstPtrs.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef VALUE_FUNCTION_DYNAMICS_H
#define VALUE_FUNCTION_DYNAMICS_H

#include <utils/types.h>
#include <utils/uncopyable.h>

#include <ros/ros.h>
#include <memory>

namespace meta {

class Dynamics : private Uncopyable {
public:
  typedef std::shared_ptr<const Dynamics> ConstPtr;

  // Destructor.
  virtual ~Dynamics() {}

  // Derived classes must be able to give the time derivative of state
  // as a function of current state and control.
  virtual VectorXd Evaluate(const VectorXd& x, const VectorXd& u) const = 0;

  // Derived classes must be able to compute an optimal control given
  // the gradient of the value function at the specified state.
  virtual VectorXd OptimalControl(const VectorXd& x,
                                  const VectorXd& value_gradient) const = 0;

  // Puncture a full state vector and return a position.
  virtual Vector3d Puncture(const VectorXd& x) const = 0;

  // Get the corresponding full state dimension to the given spatial dimension.
  virtual size_t SpatialDimension(size_t dimension) const = 0;

  // Get the min and max control in each (control) dimension.
  inline double MinControl(size_t dimension) const { return lower_u_(dimension); }
  inline double MaxControl(size_t dimension) const { return upper_u_(dimension); }

  // Get the max acceleration in the given spatial dimension.
  virtual double MaxAcceleration(size_t dimension) const = 0;

  // Derived classes must be able to translate a geometric trajectory
  // (i.e. through Euclidean space) into a full state space trajectory.
  virtual std::vector<VectorXd> LiftGeometricTrajectory(
    const std::vector<Vector3d>& positions,
    const std::vector<double>& times) const = 0;

protected:
  // Protected constructor. Use the factory method instead.
  explicit Dynamics(const VectorXd& lower_u, const VectorXd& upper_u)
    : lower_u_(lower_u),
      upper_u_(upper_u) {
    // Check that dimensions match.
    if (lower_u_.size() != upper_u_.size())
      ROS_ERROR("Upper and lower control bounds have different dimensions.");

    // Check that lower is never greater than upper.
    for (size_t ii = 0; ii < lower_u_.size(); ii++) {
      if (lower_u_(ii) > upper_u_(ii))
        ROS_ERROR("Lower control bound was above upper bound: %f > %f.",
                  lower_u_(ii), upper_u_(ii));
    }
  }

  // Lower and upper bounds for control variable.
  const VectorXd lower_u_;
  const VectorXd upper_u_;
};

} //\namespace meta

#endif
