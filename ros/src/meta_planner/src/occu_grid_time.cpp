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

#include <demo/occu_grid_time.h>

namespace meta {

// Factory method. Use this instead of the constructor.
OccuGridTime::Ptr OccuGridTime::Create(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg) {
  OccuGridTime::Ptr ptr(new OccuGridTime());
	// Set start time since this is the first callback
	ptr->start_t_ = msg->gridarray[0].header.stamp.toSec();
	// Setup the data structure from the ROS msg
	ptr->FromROSMsg(msg);
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
OccuGridTime::OccuGridTime() {}

// converts the current occupancy grid list to OccupancyGridTime msg
meta_planner_msgs::OccupancyGridTime OccuGridTime::ToROSMsg(){
  ROS_ERROR("OccuGridTime: Unimplemented method ToROSMsg.");
  return meta_planner_msgs::OccupancyGridTime();
}

// converts a given OccupancyGridTime msg to internal OccuGridTime data struct
void OccuGridTime::FromROSMsg(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg){

  // all the grids have the same height/width, so just take the first one
  height_ = msg->gridarray[0].height;
  width_ = msg->gridarray[0].width;
	resolution_ = msg->gridarray[0].resolution;

  // store the origin of the map if wasn't stored before
  if (origin_.empty()){
    origin_.push_back(msg->gridarray[0].origin.position.x);
    origin_.push_back(msg->gridarray[0].origin.position.y);
    origin_.push_back(msg->gridarray[0].origin.position.z);
  }

  // clear out old times and grids
  times_.clear();
  grids_.clear();

	//std::cout << "In FromROSMsg: gridarray size = " << msg->gridarray.size() << std::endl;

	for (size_t ii = 0; ii < msg->gridarray.size(); ii++){
    // record time stamp of current grid message
    // TODO! Add in constant offset equal to the ros::Time when the predictor started.
		const double t = msg->gridarray[ii].header.stamp.toSec();
    times_.push_back(t);

    // convert OccupancyGrid ROS message to vector of doubles
    std::vector<double> curr_grid;
    for (size_t jj = 0; jj < height_*width_; jj++){
      curr_grid.push_back(msg->gridarray[ii].data[jj]);
		}
    grids_.push_back(curr_grid);
  }

}

// Interpolate between two occupancy grids w.r.t. time
std::vector<double> OccuGridTime::InterpolateGrid(double time){

	if (start_t_ < 0.0){
		ROS_WARN("In InterpolateGrid(): times haven't been initialized!");
		return std::vector<double>();
	}

  // searches if there is a grid for the queried time.
  // if grid exists at time then lower = (idx of time) = upper
  // if does NOT exist then lower < (idx of time) < upper
  std::vector<double>::iterator it;

  // Returns iterator pointing to the first element in the range [first,last)
  // which does not compare less than time.
  it = std::lower_bound(times_.begin(), times_.end(), time);

  size_t lower, upper;
  if (it == times_.begin()){
    upper = 0; // no smaller value than time in vector
    lower = 0;
  }else if (it == times_.end()){
    lower = times_.size()-1; // no larger value than time in vector
    upper = times_.size()-1;
  }else{
    lower = std::distance(times_.begin(),it);
    upper = lower; 

    // if the value that it found does not equal time, then
    // grab the previous index for the lower bound
    if (std::abs((*it) - time) > 1e-8)
      lower--;
  }

  // if the indices are the same, then an occupancy grid already exists for
  // this time and no need to interpolate
  if (lower == upper) {
    return grids_[lower];
  }

  const double prev_t = times_[lower];
  const double next_t = times_[upper];

  const std::vector<double> prev_grid = grids_[lower];
  const std::vector<double> next_grid = grids_[upper];

  std::vector<double> interpolated_grid;

	//std::cout << "interpolated grid: "<< std::endl;
  // TODO! I think you can just do this all the time - can simplify
  // all the if-else logic above.
  for(size_t ii = 0; ii < height_ * width_; ii++){
    const double prev = prev_grid[ii];
    const double next = next_grid[ii];
    const double curr = prev + (next - prev) *
      ((time - prev_t) / (next_t - prev_t));
    interpolated_grid.push_back(curr);
  }

  return interpolated_grid;
}

size_t OccuGridTime::GetWidth() const{
  return width_;
}

size_t OccuGridTime::GetHeight() const{
  return height_;
}

double OccuGridTime::GetResolution() const{
  return resolution_;
}

double OccuGridTime::GetStartTime() const{
	return start_t_;
}

size_t OccuGridTime::GetNumGrids() const{
	return grids_.size();
}

void OccuGridTime::PrintGrid(size_t idx, bool compute_sum) const{
	if(idx >= grids_.size() || idx < 0)
		ROS_INFO("Invalid idx!\n");
	
	std::cout << "Inside PrintGrid(): Here is the grid\n";
	double sum = 0.0;
	for (size_t i = 0; i < grids_[idx].size(); i++){
		std::cout << grids_[idx][i] << " ";
		if (compute_sum)
			sum += grids_[idx][i];
	}

	if (compute_sum)
		ROS_INFO("probability sum: %f", sum);
}

// Converts a [xy] position measurement from quadcopter into grid location
std::vector<size_t> OccuGridTime::RealToSimLoc(const std::vector<double> pos,
					const Vector3d& lower, const Vector3d& upper){

	size_t locx = static_cast<size_t>(round((pos[0] - lower(0))/resolution_));
	size_t locy = static_cast<size_t>(round((upper(1) - pos[1])/resolution_));

	//ROS_INFO("pos = [%f, %f], loc = [%d, %d]", pos[0], pos[1], locx, locy);
 	std::vector<size_t> loc = {locx, locy}; 
	return loc;
}


// Takes [row,col] coordinate in simulation frame and returns a shifted
// value in the ROS coordinates
std::vector<double> OccuGridTime::SimToRealLoc(size_t row, size_t col, 
					const Vector3d& lower, const Vector3d& upper){
	std::vector<double> real = {row*resolution_ + lower(0),
															upper(1) - col*resolution_};
	return real;
}

// Converts ROS time to "real" time in seconds (double)
double OccuGridTime::ROSToRealTime(const ros::Time& rostime){
	if(start_t_ == -1) {
    ROS_ERROR("In OccuGridTime::ROSToRealTime: start_t_ was not set yet!");
		return -1.0;
	}

	return rostime.toSec() - start_t_;
}



} //\namespace meta
