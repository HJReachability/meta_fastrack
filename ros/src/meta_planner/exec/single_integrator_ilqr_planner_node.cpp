/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
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
// Node running a ILQR planner on a 3D single integrator.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/state/position_velocity.h>
#include <meta_planner/planning/ilqr_problem.h>
#include <meta_planner/planning/ilqr_solver.h>
#include <meta_planner/dynamics/single_integrator_3d.h>

#include <ros/ros.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace mp = meta_planner::planning;
namespace fs = fastrack::state;

int main(int argc, char** argv) {
  FLAGS_logtostderr = false;
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "ILQRPlanner");
  ros::NodeHandle n("~");

  auto problem =
      std::make_shared<mp::ILQRProblem<ilqgames::SingleIntegrator3D>>(n);
  mp::ILQRSolver<fs::PositionVelocity,
                 mp::ILQRProblem<ilqgames::SingleIntegrator3D>>
      solver(problem);

  if (!solver.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize ILQR.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
