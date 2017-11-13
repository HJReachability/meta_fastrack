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
// Unit tests for the ValueFunction class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/linear_dynamics.h>
#include <meta_planner/value_function.h>
#include <meta_planner/types.h>

#include <stdio.h>
#include <gtest/gtest.h>

// Test that ValueFunction initializes correctly.
TEST(ValueFunction, TestInitialize) {
  const size_t kStateDimension = 3;
  const size_t kControlDimension = 1;
  const double kControlLower = -1.0;
  const double kControlUpper = 1.0;

  // TODO! Revert to using the small test mat file.
  const std::string file_name =
    std::string(PRECOMPUTATION_DIR) + std::string("Point_3D_3D_RS.mat");
    //    std::string(PRECOMPUTATION_DIR) + std::string("test_value_function.mat");

  // Create identity dynamics.
  const Dynamics::ConstPtr dynamics = LinearDynamics::Create(
    MatrixXd::Identity(kStateDimension, kStateDimension),
    MatrixXd::Identity(kStateDimension, kControlDimension),
    VectorXd::Constant(kControlDimension, kControlLower),
    VectorXd::Constant(kControlDimension, kControlUpper));

  // Create a value function.
  ValueFunction::ConstPtr value = ValueFunction::Create(file_name, dynamics);

  // Check initialization.
  EXPECT_TRUE(value->IsInitialized());
}

// Test that we can interpolate the value function properly.
TEST(ValueFunction, TestValue) {
  const size_t kStateDimension = 3;
  const size_t kControlDimension = 1;
  const double kControlLower = -1.0;
  const double kControlUpper = 1.0;

  const std::string file_name =
    std::string(PRECOMPUTATION_DIR) + std::string("Point_3D_3D_RS.mat");
    //    std::string(PRECOMPUTATION_DIR) + std::string("test_value_function.mat");

  // Create identity dynamics.
  const Dynamics::ConstPtr dynamics = LinearDynamics::Create(
    MatrixXd::Identity(kStateDimension, kStateDimension),
    MatrixXd::Identity(kStateDimension, kControlDimension),
    VectorXd::Constant(kControlDimension, kControlLower),
    VectorXd::Constant(kControlDimension, kControlUpper));

  // Create a value function.
  ValueFunction::ConstPtr value = ValueFunction::Create(file_name, dynamics);

  // Check initialization.
  EXPECT_TRUE(value->IsInitialized());

  // Access at a specific state with known value.
  VectorXd state(VectorXd::Zero(kStateDimension));
  state(0) = -1.2;
  state(1) = -1.2;
  state(2) = -2.5761;

  const double interpolated = value->Value(state);
  const double kSmallNumber = 1e-3;
  EXPECT_NEAR(interpolated, 4.9596, kSmallNumber);
}

// Test that we can interpolate the value function properly.
TEST(ValueFunction, TestGradient) {
  const size_t kStateDimension = 3;
  const size_t kControlDimension = 1;
  const double kControlLower = -1.0;
  const double kControlUpper = 1.0;

  const std::string file_name =
    std::string(PRECOMPUTATION_DIR) + std::string("Point_3D_3D_RS.mat");
    //    std::string(PRECOMPUTATION_DIR) + std::string("test_value_function.mat");

  // Create identity dynamics.
  const Dynamics::ConstPtr dynamics = LinearDynamics::Create(
    MatrixXd::Identity(kStateDimension, kStateDimension),
    MatrixXd::Identity(kStateDimension, kControlDimension),
    VectorXd::Constant(kControlDimension, kControlLower),
    VectorXd::Constant(kControlDimension, kControlUpper));

  // Create a value function.
  ValueFunction::ConstPtr value = ValueFunction::Create(file_name, dynamics);

  // Check initialization.
  EXPECT_TRUE(value->IsInitialized());

  // Access at a specific state with known value.
  VectorXd state(VectorXd::Zero(kStateDimension));
  state(0) = 0.1;
  state(1) = 0.1;
  state(2) = 0.1;

  // Make sure gradient matches what MATLAB computes.
  const VectorXd gradient = value->Gradient(state);
  const double kSmallNumber = 1e-3;
  EXPECT_NEAR(gradient(0), 0.4295, kSmallNumber);
  EXPECT_NEAR(gradient(1), -0.24405, kSmallNumber);
  EXPECT_NEAR(gradient(2), 0.019627, kSmallNumber);
}
