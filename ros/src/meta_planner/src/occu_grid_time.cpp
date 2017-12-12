#include <demo/occu_grid_time.h>

namespace meta {

// Factory method. Use this instead of the constructor.
OccuGridTime::Ptr OccuGridTime::Create() {
  OccuGridTime::Ptr ptr(new OccuGridTime());
	ptr->start_t_ = -1;
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
OccuGridTime::OccuGridTime() {}

// converts the current occupancy grid list to OccupancyGridTime msg
meta_planner_msgs::OccupancyGridTime OccuGridTime::ToROSMsg(){
  //TODO THIS MIGHT NOT BE NEEDED AND MIGHT BE WRONG...
  meta_planner_msgs::OccupancyGridTime grid_msg;

  std::vector<meta_planner_msgs::ProbabilityGrid> msg_array;

  for (int ii = 0; ii < grids_.size(); ii++){
    meta_planner_msgs::ProbabilityGrid occu_grid;
    
    occu_grid.header.frame_id = "map";
    occu_grid.header.stamp = ros::Time::now();
    // set the time of this occupancy grid to corresponding timestamp
    occu_grid.header.stamp.sec = times_[ii];

    // populate the grid dimensions
    occu_grid.resolution = resolution_;
    occu_grid.height = height_;
    occu_grid.width = width_;
    
    geometry_msgs::Point pt;
    pt.x = origin_[0]; pt.y = origin_[1]; pt.z = origin_[2];
  
    geometry_msgs::Quaternion quat;
    quat.x = 0; quat.y = 0; quat.z = 0; quat.w = 1;

    geometry_msgs::Pose pose;
    pose.position = pt;
    pose.orientation = quat;
    occu_grid.origin = pose;

    // populate the grid data
    occu_grid.data = grids_[ii];

    // store the occupancy grid at the current time slice
    msg_array.push_back(occu_grid);
  }
  
  grid_msg.gridarray = msg_array;

  return grid_msg;
}

// converts a given OccupancyGridTime msg to internal OccuGridTime data struct
void OccuGridTime::FromROSMsg(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg){

	if (start_t_ == -1){
		// Set start time if this is the first callback
		start_t_ = msg->gridarray[0].header.stamp.toSec();
	}

  // all the grids have the same height/width, so just take the first one
  height_ = msg->gridarray[0].height;
  width_ = msg->gridarray[0].width;

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
  
	for (int ii = 0; ii < msg->gridarray.size(); ii++){
    // record time stamp of current grid message
		double t = msg->gridarray[ii].header.stamp.toSec();
    times_.push_back(t);

    // convert OccupancyGrid ROS message to vector of doubles
    std::vector<double> curr_grid;
    for (int jj = 0; jj < height_*width_; jj++){
      curr_grid.push_back(msg->gridarray[ii].data[jj]);
		}
    grids_.push_back(curr_grid);
  }

}

// Interpolate between two occupancy grids w.r.t. curr_time
std::vector<double> OccuGridTime::InterpolateGrid(double curr_time){

	if (start_t_ < 0.0){
		std::cout << "In InterpolateGrid(): times haven't been initialized!\n";
		return std::vector<double>();
	}

	std::cout << "In InterpolateGrid(): elapsed time = " << curr_time-start_t_ << std::endl;

  // searches if there is a grid for the queried time. 
  // if grid exists at curr_time then lower = (idx of curr_time) = upper
  // if does NOT exist then lower < (idx of curr_time) < upper
  std::vector<double>::iterator it;

  // Returns iterator pointing to the first element in the range [first,last) 
  // which does not compare less than curr_time.
  it = lower_bound(times_.begin(), times_.end(), curr_time);

  int lower, upper;
  if (it == times_.begin()){
    upper = 0; // no smaller value than curr_time in vector
    lower = 0;
  }else if (it == times_.end()){
    lower = times_.size()-1; // no larger value than curr_time in vector
    upper = times_.size()-1;
  }else{
    lower = std::distance(times_.begin(),it);
    upper = std::distance(times_.begin(),it);

    // if the value that it found does not equal curr_time, then 
    // grab the previous index for the lower bound    
    if ((*it) != curr_time)
      lower -= 1;
  }

	//std::cout << "lower = " << lower << std::endl;
	//std::cout << "upper = " << upper << std::endl;

  // if the indices are the same, then an occupancy grid already exists for 
  // this time and no need to interpolate
  if (lower == upper){
    return grids_[lower];
  }

  double prev_t = times_[lower];
  double next_t = times_[upper];

  std::vector<double> prev_grid = grids_[lower];
  std::vector<double> next_grid = grids_[upper];

  std::vector<double> interpolated_grid; 
 
	//std::cout << "interpolated grid: "<< std::endl;
  for(int ii = 0; ii < height_*width_; ii++){
    double prev = prev_grid[ii];
    double next = next_grid[ii];
    double curr = (next - prev)*((curr_time-prev_t)/(next_t - prev_t)) + prev;
		//std::cout << " " << curr ;
    interpolated_grid.push_back(curr);
  }
	//std::cout << "\n";

  return interpolated_grid;
}

int OccuGridTime::GetWidth() const{
  return width_;
}

int OccuGridTime::GetHeight() const{
  return height_;
}

double OccuGridTime::GetResolution() const{
  return resolution_;
}

double OccuGridTime::GetStartTime() const{
	return start_t_;
}

// Converts ROS time to "real" time in seconds (double)
double OccuGridTime::ROSToRealTime(ros::Time rostime){
	if(start_t_ == -1){
		std::cout << "In OccuGridTime::ROSToRealTime((): start_t_ was not set yet!\n";
		return -1;
	}
	return rostime.toSec() - start_t_;
}



} //\namespace meta
