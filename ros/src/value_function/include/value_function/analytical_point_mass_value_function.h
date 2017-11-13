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
 * Authors: David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 *          Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the AnalyticalPointMassValueFunction class, which inherits from
// the ValueFunction class and implements a custom OptimalControl function.
// Instead of loading subsystems, for simplicity all parameters are read
// from the ROS parameter server and this class does not utilize explicit
// subsystem classes. Assumes state order = [all positions, all velocities].
//
///////////////////////////////////////////////////////////////////////////////

#ifndef VALUE_FUNCTION_ANALYTICAL_POINT_MASS_VALUE_FUNCTION_H
#define VALUE_FUNCTION_ANALYTICAL_POINT_MASS_VALUE_FUNCTION_H

#include <value_function/value_function.h>
#include <value_function/dynamics.h>
#include <utils/types.h>
#include <utils/uncopyable.h>

#include <ros/ros.h>
#include <limits>
#include <memory>

namespace meta {

class AnalyticalPointMassValueFunction : public ValueFunction {
public:
  typedef std::shared_ptr<const AnalyticalPointMassValueFunction> ConstPtr;

  // Destructor.
  virtual ~AnalyticalPointMassValueFunction() {}

  // Factory method. Use this instead of the constructor.
  // Note that this class is const-only, which means that once it is
  // instantiated it can never be changed. Note that we must pass in
  // the maximum planner speed in each geometric dimension.
  static ConstPtr Create(const Vector3d& max_planner_speed,
                         const Vector3d& max_vel_disturbance,
                         const Vector3d& max_acc_disturbance,
                         const Vector3d& expansion_vel,
                         const Dynamics::ConstPtr& dynamics,
                         ValueFunctionId id);

  // Get velocity expansion in the subsystem containing the given spatial dim.
  double VelocityExpansion(size_t dimension) const;

  // Analytically evaluate value/gradient at a particular state.
  double Value(const VectorXd& state) const;
  VectorXd Gradient(const VectorXd& state) const;

  // Get the optimal control at a particular state.
  VectorXd OptimalControl(const VectorXd& state) const;

  // Priority of the optimal control at the given state. This is a number
  // between 0 and 1, where 1 means the final control signal should be exactly
  // the optimal control signal computed by this value function.
  double Priority(const VectorXd& state) const;

  // Get the tracking error bound in this spatial dimension.
  double TrackingBound(size_t dimension) const;

  // Get the tracker velocity bounds in this spatial dimension.
  double VelocityBound(size_t dimension) const;

  // Get the tracking error bound in this spatial dimension for a planner
  // switching INTO this one with the specified max speed.
  double SwitchingTrackingBound(
    size_t dim, const ValueFunction::ConstPtr& value) const;

  // Guaranteed time in which a planner with the specified value function
  // can switch into this value function's tracking error bound.
  double GuaranteedSwitchingTime(
    size_t dimension,
    const AnalyticalPointMassValueFunction::ConstPtr& incoming_value) const;

  // Guaranteed distance in which a planner with the specified value function
  // can switch into this value function's safe set.
  double GuaranteedSwitchingDistance(
    size_t dimension,
    const AnalyticalPointMassValueFunction::ConstPtr& incoming_value) const;

private:
  explicit AnalyticalPointMassValueFunction(const Vector3d& max_planner_speed,
                                            const Vector3d& max_vel_disturbance,
                                            const Vector3d& max_acc_disturbance,
                                            const Vector3d& expansion_vel,
                                            const Dynamics::ConstPtr& dynamics,
                                            ValueFunctionId id);

  // Reference, tracker, and disturbance parameters
  const Vector3d u_max_;            // maximum control input
  const Vector3d u_min_;            // minimum control input (not symmetric)
  const Vector3d d_v_;              // velocity disturbance
  const Vector3d d_a_;              // acceleration disturbance
  const Vector3d v_exp_;            // set expansion in velocity dimensions
  Vector3d x_exp_;                  // set expansion in position dimensions
  Vector3d a_max_;                  // maximum absolute acceleration
  Vector3d u2a_;                    // bang-bang control-to-acceleration gain

  static const size_t p_dim_;       // number of spatial dimensions (always 3)
};

} //\namespace meta

#endif
