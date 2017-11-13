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
// Unit test for MATIO.
//
///////////////////////////////////////////////////////////////////////////////

#include <matio.h>
#include <stdio.h>
#include <gtest/gtest.h>

// Test that MATIO can read in a small file correctly.
TEST(Matio, TestRead) {
  const std::string file_name =
    std::string(PRECOMPUTATION_DIR) + std::string("test.mat");
  const std::string var_name = "x";

  // Open a file pointer to this file.
  mat_t* matfp = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  ASSERT_TRUE(matfp != NULL);

  // Read the specified variable from this file.
  matvar_t* matvar = Mat_VarRead(matfp, var_name.c_str());
  ASSERT_TRUE(matvar != NULL);

  // Check content.
  ASSERT_EQ(matvar->rank, 2);
  ASSERT_EQ(matvar->dims[0], 1);
  ASSERT_EQ(matvar->dims[1], 3);
  ASSERT_EQ(matvar->isComplex, 0);
  ASSERT_EQ(matvar->data_type, MAT_T_DOUBLE);
  ASSERT_EQ(matvar->class_type, MAT_C_DOUBLE);

  const double (&data)[1][3] =
    *static_cast<const double (*)[1][3]>(matvar->data);

  for (size_t jj = 0; jj < 3; jj++)
    EXPECT_EQ(data[0][jj], static_cast<double>(jj + 1));

  // Free memory and close the file.
  Mat_VarFree(matvar);
  Mat_Close(matfp);
}
