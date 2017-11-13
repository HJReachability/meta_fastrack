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
// Unit tests for the LinearDynamics class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/linear_dynamics.h>
#include <meta_planner/types.h>

#include <stdio.h>
#include <gtest/gtest.h>

// Test that linear dynamics can determine the optimal control in a
// simple example.
TEST(LinearDynamics, TestOptimalControl) {
  const size_t kStateDimension = 10;
  const size_t kControlDimension = 10;
  const double kControlLower = -1.0;
  const double kControlUpper = 1.0;

  // Create identity dynamics.
  const Dynamics::ConstPtr dynamics = LinearDynamics::Create(
    MatrixXd::Identity(kStateDimension, kStateDimension),
    MatrixXd::Identity(kStateDimension, kControlDimension),
    VectorXd::Constant(kControlDimension, kControlLower),
    VectorXd::Constant(kControlDimension, kControlUpper));

  // Create unit value gradient.
  const VectorXd value_gradient = VectorXd::Constant(kStateDimension, 1.0);

  // Make sure optimal control is the upper bound in all dimensions.
  const VectorXd state = VectorXd::Zero(kStateDimension);
  const VectorXd optimal_control =
    dynamics->OptimalControl(state, value_gradient);

  for (size_t ii = 0; ii < kControlDimension; ii++)
    EXPECT_EQ(optimal_control(ii), kControlLower);
}
