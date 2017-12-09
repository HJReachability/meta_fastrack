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
// Defines the OmplPlanner class, which wraps the OMPL geometric planners
// and inherits from the Planner abstract class. For simplicity, we assume that
// the state space is a real-valued vector space with box constraints, i.e.
// an instance of the Box subclass of Environment.
//
// We follow these ( http://ompl.kavrakilab.org/geometricPlanningSE3.html )
// instructions for using OMPL geometric planners.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_OMPL_PLANNER_H
#define META_PLANNER_OMPL_PLANNER_H

#include <meta_planner/planner.h>
#include <meta_planner/box.h>
#include <utils/types.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <memory>

namespace meta {

namespace ob = ompl::base;
namespace og = ompl::geometric;

template<typename PlannerType>
class OmplPlanner : public Planner {
public:
  ~OmplPlanner() {}

  static Planner::Ptr Create(ValueFunctionId incoming_value,
                             ValueFunctionId outgoing_value,
                             const Box::ConstPtr& space,
                             const Dynamics::ConstPtr& dynamics);

  // Derived classes must plan trajectories between two points.
  Trajectory::Ptr Plan(const Vector3d& start,
                       const Vector3d& stop,
                       double start_time = 0.0,
                       double budget = 1.0) const;

private:
  explicit OmplPlanner(ValueFunctionId incoming_value,
                       ValueFunctionId outgoing_value,
                       const Box::ConstPtr& space,
                       const Dynamics::ConstPtr& dynamics);

  // Convert between OMPL states and Vector3ds.
  Vector3d FromOmplState(const ob::State* state) const;
};

// ------------------------------- IMPLEMENTATION --------------------------- //

template<typename PlannerType>
OmplPlanner<PlannerType>::OmplPlanner(ValueFunctionId incoming_value,
                                      ValueFunctionId outgoing_value,
                                      const Box::ConstPtr& space,
                                      const Dynamics::ConstPtr& dynamics)
  : Planner(incoming_value, outgoing_value, space, dynamics) {}

// Create OmplPlanner pointer.
template<typename PlannerType>
inline Planner::Ptr OmplPlanner<PlannerType>::
Create(ValueFunctionId incoming_value,
       ValueFunctionId outgoing_value,
       const Box::ConstPtr& space,
       const Dynamics::ConstPtr& dynamics) {
  Planner::Ptr ptr(new OmplPlanner<PlannerType>(
    incoming_value, outgoing_value, space, dynamics));
  return ptr;
}

// Derived classes must plan trajectories between two points.
template<typename PlannerType>
Trajectory::Ptr OmplPlanner<PlannerType>::
Plan(const Vector3d& start, const Vector3d& stop,
     double start_time, double budget) const {
  // Check that both start and stop are in bounds.
  if (!space_->IsValid(start, incoming_value_, outgoing_value_)) {
    ROS_WARN_THROTTLE(1.0, "Start point was in collision or out of bounds.");
    return nullptr;
  }

  if (!space_->IsValid(stop, incoming_value_, outgoing_value_)) {
    ROS_WARN_THROTTLE(1.0, "Stop point was in collision or out of bounds.");
    return nullptr;
  }

  // Create the OMPL state space corresponding to this environment.
  auto ompl_space(
    std::make_shared<ob::RealVectorStateSpace>(3));

  // Set bounds for the environment.
  const Vector3d lower = space_->LowerBounds();
  const Vector3d upper = space_->UpperBounds();

  ob::RealVectorBounds ompl_bounds(3);

  for (size_t ii = 0; ii < 3; ii++) {
    ompl_bounds.setLow(ii, lower(ii));
    ompl_bounds.setHigh(ii, upper(ii));
  }

  ompl_space->setBounds(ompl_bounds);

  // Create a SimpleSetup instance and set the state validity checker function.
	// TODO add in time when doing collision checking
	ros::Time rostime = ros::Time::now()+ros::Duration(1);
	double time = rostime.toSec();
  og::SimpleSetup ompl_setup(ompl_space);
  ompl_setup.setStateValidityChecker([&](const ob::State* state) {
      return space_->IsValid(FromOmplState(state),
                             incoming_value_, outgoing_value_, time); });

  // Set the start and stop states.
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ompl_space);
  ob::ScopedState<ob::RealVectorStateSpace> ompl_stop(ompl_space);
  for (size_t ii = 0; ii < 3; ii++) {
    ompl_start[ii] = start(ii);
    ompl_stop[ii] = stop(ii);
  }

  ompl_setup.setStartAndGoalStates(ompl_start, ompl_stop);

  // Set the planner.
  ob::PlannerPtr ompl_planner(
    new PlannerType(ompl_setup.getSpaceInformation()));
  ompl_setup.setPlanner(ompl_planner);

  // Solve. Parameter is the amount of time (in seconds) used by the solver.
  const ob::PlannerStatus solved = ompl_setup.solve(budget);

  if (solved) {
    const og::PathGeometric& solution = ompl_setup.getSolutionPath();

    // Populate the Trajectory with states and time stamps.
    std::vector<Vector3d> positions;
    std::vector<double> times;
    std::vector<ValueFunctionId> values;

    double time = start_time;
    for (size_t ii = 0; ii < solution.getStateCount(); ii++) {
      const Vector3d position = FromOmplState(solution.getState(ii));

      // Handle all other states.
      if (ii > 0) {
        const double dt = BestPossibleTime(positions.back(), position);
        time += dt;
      }

      times.push_back(time);
      positions.push_back(position);
      values.push_back(incoming_value_);
    }

    // Convert to full state space. Make sure to use the INCOMING VALUE!
    std::vector<VectorXd> full_states =
      dynamics_->LiftGeometricTrajectory(positions, times);

    return Trajectory::Create(times, full_states, values, values);
  }

  ROS_WARN("OMPL Planner could not compute a solution.");
  return nullptr;
}

// Convert between OMPL states and VectorXds.
template<typename PlannerType>
Vector3d OmplPlanner<PlannerType>::FromOmplState(
  const ob::State* state) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!state) {
    ROS_ERROR("State pointer was null.");
    return Vector3d::Zero();
  }
#endif

  const ob::RealVectorStateSpace::StateType* cast_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  Vector3d converted;
  for (size_t ii = 0; ii < 3; ii++)
    converted(ii) = cast_state->values[ii];

  return converted;
}

} //\namespace meta

#endif
