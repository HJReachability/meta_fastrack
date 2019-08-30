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
// ILQR planner.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_PLANNING_ILQR_SOLVER_H
#define META_PLANNER_PLANNING_ILQR_SOLVER_H

#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/linesearching_ilq_solver.h>
#include <ilqgames/solver/solution_splicer.h>
#include <ilqgames/utils/types.h>
#include <meta_planner/environment/balls_in_box.h>
#include <meta_planner/planning/ilqr_problem.h>

#include <fastrack/trajectory/trajectory.h>

#include <fastrack_msgs/Trajectory.h>
#include <fastrack_srvs/Replan.h>
#include <fastrack_srvs/ReplanRequest.h>
#include <fastrack_srvs/ReplanResponse.h>
#include <fastrack_srvs/TrackingBoundBox.h>
#include <fastrack_srvs/TrackingBoundBoxResponse.h>

#include <ros/ros.h>

namespace meta_planner {
namespace planning {

using namespace ilqgames;

template <typename S, typename P>
class ILQRSolver {
 public:
  ~ILQRSolver() {}
  ILQRSolver(const std::shared_ptr<P>& problem)
      : problem_(problem), initialized_(false) {}

  // Initialize from a node handle.
  bool Initialize(const ros::NodeHandle& n);

 private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback to handle replanning requests.
  bool ReplanServer(fastrack_srvs::ReplanRequest& req,
                    fastrack_srvs::ReplanResponse& res) {
    // Unpack start/stop states.
    const S start(req.req.start);
    const S goal(req.req.goal);

    // HACK! Assume D states are just position.
    const Vector3d start_position = start.Position();
    const Vector3d goal_position = goal.Position();

    // Plan.
    const fastrack::trajectory::Trajectory<S> traj =
        Plan(VectorXf(start_position.cast<float>()),
             VectorXf(goal_position.cast<float>()), req.req.start_time);

    // Return whether or not planning was successful.
    res.traj = traj.ToRos();
    return traj.Size() > 0;
  }

  // Plan a trajectory from the given start to goal states starting
  // at the given time.
  fastrack::trajectory::Trajectory<S> Plan(const VectorXf& start,
                                           const VectorXf& goal,
                                           double start_time = 0.0) const;

  // Problem.
  std::shared_ptr<P> problem_;

  // Max amount of time for planning to run each time.
  float max_runtime_;

  // Replanning server.
  ros::ServiceServer replan_srv_;
  std::string replan_srv_name_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};  //\class ILQRSolver

// ----------------------------- IMPLEMENTATION ----------------------------- //

template <typename S, typename P>
fastrack::trajectory::Trajectory<S> ILQRSolver<S, P>::Plan(
    const VectorXf& start, const VectorXf& goal, double start_time) const {
  // HACK! Assuming state layout.
  const float final_x = static_cast<float>(goal(0));
  const float final_y = static_cast<float>(goal(1));
  const float final_z = static_cast<float>(goal(2));

  // const double call = ros::Time::now().toSec();
  // // problem_->SetUpNextRecedingHorizon(initial.cast<float>(), start_time,
  // //                                    max_runtime_);
  problem_->SetUpCosts(start, final_x, final_y, final_z, start_time);
  const auto log = problem_->Solve(max_runtime_);
  //  CHECK_NOTNULL(log.get());

  // ROS_INFO("%s: planning time was %f seconds.", name_.c_str(),
  //          ros::Time::now().toSec() - call);

  // Parse into trajectory.
  std::vector<S> states;
  std::vector<double> times;

  const OperatingPoint& op = problem_->CurrentOperatingPoint();

  double t = op.t0;
  for (size_t ii = 0; ii < op.xs.size(); ii++) {
    states.push_back(S(op.xs[ii].cast<double>()));
    times.push_back(t +
                    static_cast<double>(ii) * problem_->Solver().TimeStep());

    std::cout << "t = " << times.back() << ": " << op.xs[ii].transpose()
              << std::endl;
  }

  return fastrack::trajectory::Trajectory<S>(states, times);
}

template <typename S, typename P>
bool ILQRSolver<S, P>::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Planner");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
template <typename S, typename P>
bool ILQRSolver<S, P>::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Services.
  if (!nl.getParam("srv/replan", replan_srv_name_)) return false;

  // Max runtime per call.
  if (!nl.getParam("max_runtime", max_runtime_)) return false;

  return true;
}

// Register callbacks.
template <typename S, typename P>
bool ILQRSolver<S, P>::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Services.
  replan_srv_ = nl.advertiseService(replan_srv_name_.c_str(),
                                    &ILQRSolver<S, P>::ReplanServer, this);

  return true;
}

}  //\namespace planning
}  //\namespace fastrack

#endif
