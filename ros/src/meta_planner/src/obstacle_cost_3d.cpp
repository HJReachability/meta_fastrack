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
// Penalizes (thresh - relative distance)^2 between state and fixed obstacle
// whenever relative distance is less than thresh.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/utils/types.h>
#include <meta_planner/planning/obstacle_cost_3d.h>

#include <glog/logging.h>

namespace ilqgames {

float ObstacleCost3D::Evaluate(const VectorXf& input) const {
  const float dx = input(xidx_) - obstacle_.x();
  const float dy = input(yidx_) - obstacle_.y();
  const float dz = input(zidx_) - obstacle_.z();
  const float delta_sq = dx * dx + dy * dy + dz * dz;

  if (delta_sq >= threshold_sq_) return 0.0;

  const float gap = threshold_ - std::sqrt(delta_sq);
  return 0.5 * weight_ * gap * gap;
}

void ObstacleCost3D::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                VectorXf* grad) const {
  CHECK_NOTNULL(hess);

  // Check dimensions.
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());

  if (grad) CHECK_EQ(input.size(), grad->size());

  // Compute Hessian and gradient.
  const float dx = input(xidx_) - obstacle_.x();
  const float dy = input(yidx_) - obstacle_.y();
  const float dz = input(zidx_) - obstacle_.z();
  const float delta_sq = dx * dx + dy * dy + dz * dz;

  // Catch cost not active.
  if (delta_sq >= threshold_sq_) return;

  std::cout << "cost is " << Evaluate(input) << std::endl;

  const float delta = std::sqrt(delta_sq);
  const float gap = threshold_ - delta;
  const float weight_delta = weight_ / delta;
  const float dx_delta = dx / delta;
  const float dy_delta = dy / delta;
  const float dz_delta = dz / delta;

  const float hess_xx = weight_delta * (dx_delta * (gap * dx_delta + dx) - gap);
  (*hess)(xidx_, xidx_) += hess_xx;

  const float hess_yy = weight_delta * (dy_delta * (gap * dy_delta + dy) - gap);
  (*hess)(yidx_, yidx_) += hess_yy;

  const float hess_zz = weight_delta * (dz_delta * (gap * dz_delta + dz) - gap);
  (*hess)(zidx_, zidx_) += hess_yy;

  const float hess_xy = weight_delta * (dx_delta * (gap * dy_delta + dy));
  (*hess)(xidx_, yidx_) += hess_xy;
  (*hess)(yidx_, xidx_) += hess_xy;

  const float hess_xz = weight_delta * (dx_delta * (gap * dz_delta + dz));
  (*hess)(xidx_, zidx_) += hess_xz;
  (*hess)(zidx_, xidx_) += hess_xz;

  const float hess_yz = weight_delta * (dy_delta * (gap * dz_delta + dz));
  (*hess)(yidx_, zidx_) += hess_yz;
  (*hess)(zidx_, yidx_) += hess_yz;

  if (grad) {
    const float ddx = -weight_delta * gap * dx;
    (*grad)(xidx_) += ddx;

    const float ddy = -weight_delta * gap * dy;
    (*grad)(yidx_) += ddy;

    const float ddz = -weight_delta * gap * dz;
    (*grad)(zidx_) += ddz;
  }
}

}  // namespace ilqgames
