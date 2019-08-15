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

#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/linesearching_ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/types.h>

#include <fastrack_srvs/TrackingBoundBox.h>
#include <fastrack_srvs/TrackingBoundBoxResponse.h>
#include <meta_planner/environment/balls_in_box.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace meta_planner {
namespace planning {

using namespace ilqgames;

class ILQRProblem {
 public:
  ~ILQRProblem() {}

  template <typename D>
  ILQRProblem(const ros::NodeHandle& n);

  // Update goal location.
  void UpdateGoal(const VectorXf& x) {
    // TODO!
  }

 private:
  // Load parameters.
  void LoadParameters(const ros::NodeHandle& n);

  // Environment and range to pad obstacles.
  meta_planner::environment::BallsInBox env_;
  float max_tracking_error_;
};  //\class ILQR

// ----------------------------- IMPLEMENTATION ----------------------------- //

template <typename D>
ILQRProblem<D>::ILQRProblem(const ros::NodeHandle& n) {
  LoadParameters(n);
  CHECK(env_.Initialize(n));

  // TODO!
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

  // Costs.
  // TODO!

  return true;
}

}  //\namespace planning
}  //\namespace fastrack

#endif
