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
// Defines an n-dimensional box which inherits from Environment. Defaults to
// the unit box.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_BOX_H
#define META_PLANNER_BOX_H

#include <meta_planner/environment.h>

#include <ros/ros.h>
#include <memory>
#include <algorithm>
#include <random>

namespace meta {

class Box : public Environment {
public:
  typedef std::shared_ptr<Box> Ptr;
  typedef std::shared_ptr<const Box> ConstPtr;

  // Factory method. Use this instead of the constructor.
  static Ptr Create();

  // Destructor.
  virtual ~Box() {}

  // Inherited from Environment, but can be overwritten by child classes.
  virtual Vector3d Sample() const;

  // Inherited from Environment, but can be overwritten by child classes.
  // Returns true if the state is a valid configuration.
  // Takes in incoming and outgoing value functions. See planner.h for details.
  virtual bool IsValid(const Vector3d& position,
                       ValueFunctionId incoming_value,
                       ValueFunctionId outgoing_value,
                       double time=-1) const;

  // Inherited by Environment, but can be overwritten by child classes.
  // Assumes that the first <=3 dimensions correspond to R^3.
  virtual void Visualize(const ros::Publisher& pub,
                         const std::string& frame_id) const;

  // Set bounds in each dimension.
  void SetBounds(const Vector3d& lower, const Vector3d& upper);

  // Get the dimension and upper/lower bounds as const references.
  inline const Vector3d& LowerBounds() const { return lower_; }
  inline const Vector3d& UpperBounds() const { return upper_; }

protected:
  explicit Box();

  // Bounds.
  Vector3d lower_;
  Vector3d upper_;
};

} //\namespace meta

#endif
