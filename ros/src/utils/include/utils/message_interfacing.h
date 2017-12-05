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
// Helper functions to pack and unpack different messages.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UTILS_MESSAGE_INTERFACING_H
#define UTILS_MESSAGE_INTERFACING_H

#include <utils/types.h>
#include <meta_planner_msgs/State.h>
#include <meta_planner_msgs/Control.h>

#include <geometry_msgs/Vector3.h>

namespace meta {
namespace utils {

// Unpack a Vector3 into a Vector3d.
inline Vector3d Unpack(const geometry_msgs::Vector3& msg) {
  return Vector3d(msg.x, msg.y, msg.z);
}

// Pack a Vector3d into a Vector3.
inline geometry_msgs::Vector3 Pack(const Vector3d& point) {
  geometry_msgs::Vector3 msg;
  msg.x = point(0);
  msg.y = point(1);
  msg.z = point(2);

  return msg;
}

// Unpack a State message into a VectorXd.
inline VectorXd Unpack(const meta_planner_msgs::State& msg) {
  VectorXd state(msg.dimension);
  for (size_t ii = 0; ii < state.size(); ii++)
    state(ii) = msg.state[ii];

  return state;
}

// Pack a VectorXd into a State message.
inline meta_planner_msgs::State PackState(const VectorXd& state) {
  meta_planner_msgs::State msg;
  msg.dimension = state.size();
  for (size_t ii = 0; ii < state.size(); ii++)
    msg.state.push_back(state(ii));

  return msg;
}

// Unpack a Control message into a VectorXd.
inline VectorXd Unpack(const meta_planner_msgs::Control& msg) {
  VectorXd control(msg.dimension);
  for (size_t ii = 0; ii < control.size(); ii++)
    control(ii) = msg.control[ii];

  return control;
}

// Pack a VectorXd into a Control message.
inline meta_planner_msgs::Control PackControl(const VectorXd& control) {
  meta_planner_msgs::Control msg;
  msg.dimension = control.size();
  for (size_t ii = 0; ii < control.size(); ii++)
    msg.control.push_back(control(ii));

  return msg;
}

} //\namespace utils

} //\namespace meta

#endif
