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
// subsystem classes.
//
///////////////////////////////////////////////////////////////////////////////

#include <value_function/analytical_point_mass_value_function.h>

namespace meta {

const size_t AnalyticalPointMassValueFunction::p_dim_ = 3;

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed. Note that we must pass in
// the maximum planner speed in each geometric dimension.
AnalyticalPointMassValueFunction::ConstPtr AnalyticalPointMassValueFunction::
Create(const Vector3d& max_planner_speed,
       const Vector3d& max_vel_disturbance,
       const Vector3d& max_acc_disturbance,
       const Vector3d& expansion_vel,
       const Dynamics::ConstPtr& dynamics,
       ValueFunctionId id) {
  AnalyticalPointMassValueFunction::ConstPtr ptr(
    new AnalyticalPointMassValueFunction(max_planner_speed, max_vel_disturbance,
                                         max_acc_disturbance, expansion_vel,
                                         dynamics, id));
  return ptr;
}

// Get velocity expansion in the subsystem containing the given spatial dim.
double AnalyticalPointMassValueFunction::VelocityExpansion(size_t dimension) const {
  return v_exp_(dimension);
}

// Analytically evaluate value/gradient at a particular state.
double AnalyticalPointMassValueFunction::
Value(const VectorXd& state) const {
  double V = -std::numeric_limits<double>::infinity();
  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_ + dim);
    const double v_ref = max_planner_speed_(dim);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) - x_exp_(dim);
    //    const double V_A = ( 1/2*pow(v-v_ref,2) - pow(v_ref,2) )
    //      /   ( a_max_(dim) - d_a_(dim) ) - x;

    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) + x_exp_(dim);
    //    const double V_B = x - ( -1/2*pow(v+v_ref,2) + pow(v_ref,2) )
    //      /   ( a_max_(dim) - d_a_(dim) );

    // Value function is the maximum of the above two surfaces.
    const double V_this_dim = std::max(V_A, V_B);
    V = std::max(V, V_this_dim);
  }

  return V;
}

VectorXd AnalyticalPointMassValueFunction::
Gradient(const VectorXd& state) const {
  VectorXd grad_V = VectorXd::Zero(x_dim_);

  // Loop through each subsystem and populate grad_V.
  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_ + dim);
    const double v_ref = max_planner_speed_(dim);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) - x_exp_(dim);
    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) + x_exp_(dim);
    if (V_A > V_B) {
      grad_V(dim) = -1.0;         // if on A side, grad points towards -pos
      grad_V(p_dim_ + dim) = (v - v_ref) / (a_max_(dim) - d_a_(dim));
    } else {
      grad_V(dim) = 1.0;          // if on B side, grad points towards +pos
      grad_V(p_dim_ + dim) = (v + v_ref) / (a_max_(dim) - d_a_(dim));
    }
  }

  return grad_V;
}

// Get the optimal control at a particular state.
VectorXd AnalyticalPointMassValueFunction::
OptimalControl(const VectorXd& state) const {
  Vector3d u_opt = Vector3d::Zero();

  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_+dim);
    const double v_ref = max_planner_speed_(dim);
    VectorXd grad_V = VectorXd::Zero(x_dim_);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) - x_exp_(dim);
    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim)) + x_exp_(dim);
    const double V = std::max(V_A,V_B);
    // Determine acceleration and deceleration input in this dimension
    const double u_acc = u2a_(dim) > 0.0 ? u_max_(dim) : u_min_(dim);
    const double u_dec = u2a_(dim) > 0.0 ? u_min_(dim) : u_max_(dim);
    // Inside rule
    //    if (V <= 0)
    if (false)
      u_opt(dim) = (V_A > V_B) ? u_acc : u_dec;
    // Outside rule
    else {
      if (x >= 0) // If A-curve can catch you brake, else accelerate.
        u_opt(dim) = (V_A < 0) ? u_dec : u_acc;
      else // If B-curve can catch you accelerate, else brake.
        u_opt(dim) = (V_B < 0) ? u_acc : u_dec;
    } // if V <= 0, else
  } // for dim

  return u_opt;
}

// Priority of the optimal control at the given state. This is a number
// between 0 and 1, where 1 means the final control signal should be exactly
// the optimal control signal computed by this value function.
double AnalyticalPointMassValueFunction::
Priority(const VectorXd& state) const {
  const double V = Value(state);

  // HACK! The threshold should probably be externally set via config.
  const double relative_high = 0.20; // 10% of max inside value
  const double relative_low  = 0.05; // 5% of max inside value

  const double V_safest = Value(VectorXd::Zero(6));

  // BUG! @JFF this needs to be multiplying the MAX V in the set, not the MIN.
  const double V_high = relative_high * V_safest;
  const double V_low = relative_low * V_safest;

  const double priority = 1.0 -
                          std::min(std::max(0.0,(V-V_low)/(V_high-V_low)),1.0);
  return priority;
}

// Get the tracking error bound in this spatial dimension.
double AnalyticalPointMassValueFunction::
TrackingBound(size_t dim) const {
  // Return a single positive number (semi-length of interval centered on 0)
  // This is equal to the position at the intersection between parabolas.
  const double v_ref = max_planner_speed_(dim);

  return (v_ref+d_v_(dim))*(v_ref+d_v_(dim))
          / (a_max_(dim) - d_a_(dim)) + x_exp_(dim);
}

// Get the tracker velocity bounds in this spatial dimension.
double AnalyticalPointMassValueFunction::
VelocityBound(size_t dim) const{
  // Return a single positive number (semi-length of interval centered on 0)
  // This is equal to the position at the intersection between parabolas.
  const double v_ref = max_planner_speed_(dim);

  return std::sqrt(v_ref*v_ref + 2*(a_max_(dim) - d_a_(dim))*x_exp_(dim));
}

// Get the tracking error bound in this spatial dimension for a planner
// switching INTO this one with the specified max speed.
double AnalyticalPointMassValueFunction::
SwitchingTrackingBound(size_t dim, const ValueFunction::ConstPtr& value) const {
  // For point-mass-to-point-mass the switching error bound is the same in
  // position dimensions as the tracking error bound of the incoming planner.
  return value->TrackingBound(dim);
}

// Guaranteed time in which a planner with the specified value function
// can switch into this value function's tracking error bound.
double AnalyticalPointMassValueFunction::
GuaranteedSwitchingTime(
  size_t dim,
  const AnalyticalPointMassValueFunction::ConstPtr& value_in) const {
  const double a_max = a_max_(dim);
  const double d_a = d_a_(dim);
  const double v_ref = max_planner_speed_(dim);
  const double v_exp = v_exp_(dim);
  const double x_exp = x_exp_(dim);
  const double v_ref_in = value_in->max_planner_speed_(dim);
  const double a_max_in = value_in->a_max_(dim);
  const double d_a_in = value_in->d_a_(dim);
  const double v_bound_in = value_in->VelocityBound(dim);

  // Velocity coordinate at which to switch into entry parabola starting on
  // the boundary of the incoming tracking error bound (worst case).
  const double v_switch = std::sqrt(
               (a_max - d_a)*(x_exp + v_ref_in*v_bound_in)/(a_max_in - d_a_in)
               +  0.5 * (v_bound_in + v_ref) * (v_bound_in + v_ref) );

  // Upper bound on time to reach the entry parabola and switch onto it.
  const double t_switch = (v_bound_in + v_switch) / ((a_max- d_a));
  // Upper bound on time to ride the entry parabola into the set.
  const double t_ride = (value_in->TrackingBound(dim) - TrackingBound(dim))
    / v_exp;

  return t_switch + t_ride;
}

// Guaranteed distance in which a planner with the specified value function
// can switch into this value function's tracking error bound.
double AnalyticalPointMassValueFunction::
GuaranteedSwitchingDistance(
  size_t dim,
  const AnalyticalPointMassValueFunction::ConstPtr& value_in) const {
  // HACK! Assume value_in is of class analytic_point_mass_value_function
  // so that private max_planner_speed_ can be read from this function
  return max_planner_speed_(dim) *
         GuaranteedSwitchingTime(dim, value_in);
}

// Constructor.
AnalyticalPointMassValueFunction::
AnalyticalPointMassValueFunction(const Vector3d& max_planner_speed,
                                 const Vector3d& max_vel_disturbance,
                                 const Vector3d& max_acc_disturbance,
                                 const Vector3d& expansion_vel,
                                 const Dynamics::ConstPtr& dynamics,
                                 ValueFunctionId id)
  : ValueFunction(dynamics, 6, 3, id),
    u_max_(Vector3d(dynamics->MaxControl(0),
                    dynamics->MaxControl(1),
                    dynamics->MaxControl(2))),
    u_min_(Vector3d(dynamics->MinControl(0),
                    dynamics->MinControl(1),
                    dynamics->MinControl(2))),
    d_v_(max_vel_disturbance),
    d_a_(max_acc_disturbance),
    v_exp_(expansion_vel) {
  // Compute max acceleration.
  a_max_(0) = dynamics->MaxAcceleration(0);
  a_max_(1) = dynamics->MaxAcceleration(1);
  a_max_(2) = dynamics->MaxAcceleration(2);

  // Compute control gains.
  // NOTE: assumed symmetric even if u_max != -u_min.
  const VectorXd x_dot_max = dynamics_->Evaluate(VectorXd::Zero(6), u_max_);
  u2a_ = x_dot_max.tail<3>().cwiseQuotient( 0.5 * (u_max_ - u_min_) );

  // Max planner speed
  max_planner_speed_ = max_planner_speed;

  // Expansion of set boundaries in the position dimension
  x_exp_ = expansion_vel.cwiseProduct(2*max_planner_speed + 0.5*expansion_vel)
            .cwiseQuotient(a_max_ - d_a_);
}

} //\namespace meta
