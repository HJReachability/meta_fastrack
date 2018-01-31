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

#ifndef META_PLANNER_OCCU_GRID_TIME_H
#define META_PLANNER_OCCU_GRID_TIME_H

//#include <meta_planner/types.h>
#include <meta_planner_msgs/OccupancyGridTime.h>
#include <meta_planner_msgs/ProbabilityGrid.h>

#include <utils/types.h>
#include <ros/ros.h>
#include <vector>

namespace meta {

class OccuGridTime {

public:
  typedef std::shared_ptr<OccuGridTime> Ptr;
  typedef std::shared_ptr<const OccuGridTime> ConstPtr;

  // Factory method. Use this instead of the constructor.
  static Ptr Create(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg);

  // Destructor.
  ~OccuGridTime() {}

  // converts the current occupancy grid list to OccupancyGridTime msg
  meta_planner_msgs::OccupancyGridTime ToROSMsg();

  // converts a given OccupancyGridTime msg to size_ternal OccuGridTime data struct
  void FromROSMsg(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg);

  // given a time, returns an size_terpolated flattened 1D occupancy grid
  double InterpolateGrid(double time, 
    std::vector<size_t> min_loc, std::vector<size_t> max_loc);

  size_t GetWidth() const;
  size_t GetHeight() const;
  double GetResolution() const;
	double GetStartTime() const;
	size_t GetNumGrids() const;

	std::vector<size_t> RealToSimLoc(const std::vector<double> pos, 
					const Vector3d& lower, const Vector3d& upper);

	std::vector<double> SimToRealLoc(size_t row, size_t col, 
					const Vector3d& lower, const Vector3d& upper);

	// Converts ROS time to "real" time in seconds (double)
	double ROSToRealTime(const ros::Time& rostime);

	// Prints the contents of one occupancy grid at idx and also the total prob
	void PrintGrid(size_t idx, bool compute_sum) const;

private:
  explicit OccuGridTime();

  // dimensions of each of the grids
  size_t height_;
  size_t width_;
  double resolution_;
  std::vector<double> origin_;

	// Stores ROS time from the first OccupancyGrid msg. 
	double start_t_;

  // stores list of "flattened" 1D occupancy grids
  std::vector<std::vector<double> > grids_;

  // stores corresponding times for each of the grids
  std::vector<double> times_;

};

} //\namespace meta

#endif
