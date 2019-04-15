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
// Defines a Box environment with spherical obstacles. For simplicity, this
// does not bother with a kdtree index to speed up collision queries, since
// it is only for a simulated demo.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DEMO_BALLS_IN_BOX_H
#define DEMO_BALLS_IN_BOX_H

#include <meta_planner/box.h>
#include <utils/types.h>

#include <vector>

namespace meta {

class BallsInBox : public Box {
public:
  typedef std::shared_ptr<BallsInBox> Ptr;
  typedef std::shared_ptr<const BallsInBox> ConstPtr;

  // Factory method. Use this instead of the constructor.
  static Ptr Create();

  // Destructor.
  ~BallsInBox() {}

  // Inherited collision checker from Box needs to be overwritten.
  // Takes in incoming and outgoing value functions. See planner.h for details.
  bool IsValid(const Vector3d& position,
               ValueFunctionId incoming_value,
               ValueFunctionId outgoing_value) const;

  // Check for obstacles within a sensing radius. Returns true if at least
  // one obstacle was sensed.
  bool SenseObstacles(const Vector3d& position, double sensor_radius,
                      std::vector<Vector3d>& obstacle_positions,
                      std::vector<double>& obstacle_radii) const;

  // Check if a given obstacle is in the environment.
  bool IsObstacle(const Vector3d& obstacle_position,
                  double obstacle_radius);

  // Inherited visualizer from Box needs to be overwritten.
  void Visualize(const ros::Publisher& pub, const std::string& frame_id) const;

  // Add a spherical obstacle of the given radius to the environment.
  void AddObstacle(const Vector3d& point, double r);

private:
  BallsInBox();

  // List of obstacle locations and radii.
  std::vector<VectorXd> points_;
  std::vector<double> radii_;
};

} //\namespace meta

#endif
