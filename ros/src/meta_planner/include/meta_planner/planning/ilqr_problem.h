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
// ILQR problem.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_PLANNING_ILQR_PROBLEM_H
#define META_PLANNER_PLANNING_ILQR_PROBLEM_H

#include <ilgames/cost/quadratic_cost.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/linesearching_ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/types.h>
#include <meta_planner/planning/obstacle_cost_3d.h>

#include <fastrack_srvs/TrackingBoundBox.h>
#include <fastrack_srvs/TrackingBoundBoxResponse.h>
#include <meta_planner/environment/balls_in_box.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace meta_planner {
namespace planning {

namespace {
// Time.
static constexpr Time kTimeStep = 0.1;      // s
static constexpr Time kTimeHorizon = 10.0;  // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Flag for one-sided costs.
static constexpr bool kOrientedRight = true;
}  // anonymous namespace

using namespace ilqgames;

class ILQRProblem : public Problem {
 public:
  ~ILQRProblem() {}

  template <typename D>
  ILQRProblem(const ros::NodeHandle& n);

  // Reset costs. Include new goal location at the given coordinates.
  void SetUpCosts(float goal_x, float goal_y, float goal_z);

 private:
  // Load parameters.
  void LoadParameters(const ros::NodeHandle& n);

  // Environment and range to pad obstacles.
  meta_planner::environment::BallsInBox env_;
  float max_tracking_error_;

  // Dynamics.
  std::shared_ptr<ConcatenatedDynamicalSystem> dynamics_;

  // Cost weights.
  float goal_cost_weight_;
  float control_effort_cost_weight_;
  float obstacle_cost_weight_;
};  //\class ILQR

// ----------------------------- IMPLEMENTATION ----------------------------- //

template <typename D>
ILQRProblem<D>::ILQRProblem(const ros::NodeHandle& n) {
  LoadParameters(n);
  CHECK(env_.Initialize(n));

  // Create dynamics.
  dynamics_.reset(new ConcatenatedDynamicalSystem({
      std::make_shared<D>(),
  }));

  // Set up initial state.
  // NOTE: this will get overwritten before the solver is actually called.
  x0_ = VectorXf::Constant(dynamics->XDim(),
                           std::numeric_limits<float>::quiet_NaN());

  // Set up initial strategies and operating point.
  strategies_.reset(new std::vector<Strategy>());
  for (size_t ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players.
  SetUpCosts();
}

template <typename D>
void SetUpCosts(float goal_x, float goal_y, float goal_z) {
  PlayerCost p1_cost;

  // Penalize control effort.
  const auto p1_u_cost = std::make_shared<QuadraticCost>(
      control_effort_cost_weight_, -1, 0.0, "Control Effort");
  p1_cost.AddControlCost(0, p1_u_cost);

  // Goal costs.
  constexpr float kFinalTimeWindow = 0.5;  // s
  const auto p1_goalx_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(goal_cost_weight_, D::kPxIdx, goal_x),
      kTimeHorizon - kFinalTimeWindow, "GoalX");
  const auto p1_goaly_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(goal_cost_weight_, D::kPyIdx, goal_y),
      kTimeHorizon - kFinalTimeWindow, "GoalY");
  const auto p1_goalz_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(goal_cost_weight_, D::kPzIdx, goal_z),
      kTimeHorizon - kFinalTimeWindow, "GoalZ");
  p1_cost.AddStateCost(p1_goalx_cost);
  p1_cost.AddStateCost(p1_goaly_cost);
  p1_cost.AddStateCost(p1_goalz_cost);

  // Obstacle costs.
  const auto& centers = env_.Centers();
  const auto& radii = env_.Radii();

  for (size_t ii = 0; ii < env_.NumObstacles(); ii++) {
    // HACK! Set avoidance threshold to twice radius.
    const std::shared_ptr<ObstacleCost> obstacle_cost(new ObstacleCost3D(
        obstacle_cost_weight_, {D::kPxIdx, D::kPyIdx, D::kPzIdx},
        centers[ii].cast<float>(), 2.0 * radii[ii],
        "Obstacle" + std::to_string(ii)));
    p1_cost.AddStateCost(obstacle_cost);
  }

  // Create the corresponding solver.
  solver_.reset(new LinesearchingILQSolver(dynamics_, {p1_cost}, kTimeHorizon,
                                           kTimeStep));
}

inline void Planner<S, D>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Set bound by calling service provided by tracker.
  std::string bound_srv_name;
  if (!nl.getParam("srv/bound", bound_srv_name)) return false;

  ros::service::waitForService(bound_srv_name);
  ros::ServiceClient bound_srv =
      nl.serviceClient<fastrack_srvs::TrackingBoundBox>(bound_srv_name.c_str(),
                                                        true);

  fastrack_srvs::TrackingBoundBox b;
  CHECK(bound_srv);
  CHECK(bound_srv.call(b));

  fastrack::bound::Box bound;
  bound.FromRos(b.response);
  max_tracking_error_ = std::hypot(bound.x, bound.y, bound.z);

  // Cost weights.
  if (!nl.getParam("weight/obstacle", obstacle_cost_weight_)) return false;
  if (!nl.getParam("weight/goal", goal_cost_weight_)) return false;
  if (!nl.getParam("weight/control_effort", control_effort_cost_weight_))
    return false;

  return true;
}

}  //\namespace planning
}  //\namespace fastrack

#endif
