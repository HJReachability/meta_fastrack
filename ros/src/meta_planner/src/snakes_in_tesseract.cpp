///////////////////////////////////////////////////////////////////////////////
//
// Defines a Box environment with time-varying cube obstacles that represent
// locations that are very likely to have a human at them.
//
///////////////////////////////////////////////////////////////////////////////

#include <demo/snakes_in_tesseract.h>

namespace meta {

// Factory method. Use this instead of the constructor.
SnakesInTesseract::Ptr SnakesInTesseract::Create() {
  SnakesInTesseract::Ptr ptr(new SnakesInTesseract());
  ptr->occu_grids_ = nullptr;
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
SnakesInTesseract::SnakesInTesseract()
  : ProbabilisticBox() {}

// Load topics and probability thresholds
bool SnakesInTesseract::LoadParameters(const ros::NodeHandle& n){

  ros::NodeHandle nl(n);

	std::cout << "In load params" << std::endl;

	// Sensor radius.
  if (!nl.getParam("srv/switching_bound", switching_bound_name_)) return false;

  // Occupancy Grid data and marker topics.
  if (!nl.getParam("topics/occupancy_grid_time", occu_grid_topic_)) return false;
	if (!nl.getParam("topics/occu_grid_marker", occu_grid_marker_topic_)) return false;

  if (!nl.getParam("frames/fixed", fixed_frame_id_)) return false;

	if (!nl.getParam("topics/trigger_replan", trigger_replan_topic_)) return false;

 	// Probability threshold.
  if (!nl.getParam("prob_thresh", threshold_)) return false;

	return true;
}

// Register all callbacks and publishers.
bool SnakesInTesseract::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

 	// Server.
  switching_bound_srv_ = nl.serviceClient<value_function::SwitchingTrackingBoundBox>(
    switching_bound_name_.c_str(), true);

  // Subscriber.
  occu_grid_sub_ = nl.subscribe(
    occu_grid_topic_.c_str(), 1, &SnakesInTesseract::OccuGridCallback, this);

  // Publisher for occupancy grid markers
  occu_grid_marker_pub_ = nl.advertise<visualization_msgs::Marker>(
    occu_grid_marker_topic_.c_str(), 10, false);

  // Triggering a replan event.
  trigger_replan_pub_ = nl.advertise<std_msgs::Empty>(
    trigger_replan_topic_.c_str(), 1, false);

  return true;
}

void SnakesInTesseract::OccuGridCallback(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg){
	if (occu_grids_ == nullptr){
		// Construct occugrid if doesn't exist yet
		occu_grids_ = OccuGridTime::Create(msg);
	}else{
		// update and convert the incoming message to OccuGridTime data structure
		occu_grids_->FromROSMsg(msg);
	}

	// print the contents of the first grid entry for debug
	//occu_grids_->PrintGrid(0, true);

	//ROS_INFO("I've gotten new occupancy grid data! Triggering replan.");

	// TODO REMOVE THIS HACK!!
	double curr_time = ros::Time::now().toSec();
	if (abs(curr_time-last_traj_request_) > 1.5){
		trigger_replan_pub_.publish(std_msgs::Empty());
		last_traj_request_ = curr_time;
	}	

	//TODO THIS IS JUST FOR DEBUGGING RIGHT NOW (?)
	//ros::Time now = ros::Time::now();
	//VisualizeOccuGrid(now, 1);
	//VisualizeOccuGrid(now, 2);
	//VisualizeOccuGrid(now, 3);
}

// Inherited collision checker from Box needs to be overwritten.
// Takes in incoming and outgoing value functions and sample time (in the future).
bool SnakesInTesseract::IsValid(const Vector3d& position,
                         ValueFunctionId incoming_value,
                         ValueFunctionId outgoing_value,
                         double& collision_prob, double time) const {
  ros::Time start = ros::Time::now();
  // compute the total likelihood of collision for the current position
  collision_prob = CollisionProbability(position, 
    incoming_value, outgoing_value, time);

  double elapsed = (ros::Time::now() - start).toSec();

  ROS_INFO("elapsed time for collision check: %f", elapsed);

  // if the summed probability above threshold of collision, return not valid
  if (collision_prob > threshold_){
		//ROS_WARN("Collision prob (%f) > threshold (%f)", collision_prob, threshold_);
    return false;
	}

  return true;
}

// Returns the (total) collision probability at position.
double SnakesInTesseract::CollisionProbability(const Vector3d& position,
             ValueFunctionId incoming_value,
             ValueFunctionId outgoing_value,
             double time) const{
#ifdef ENABLE_DEBUG_MESSAGES
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check an uninitialized SnakesInTesseract.",
             name_.c_str());
    return 1.0;
  }
#endif

  // Make sure valid time is being passed in.
  if (time < 0.0) {
    ROS_WARN_THROTTLE(1.0, "%s: Tried to collision check for negative time.",
                      name_.c_str());
    return 1.0;
  }

  if (occu_grids_ == nullptr) {
    ROS_WARN_THROTTLE(1.0, "%s: Tried to collision check before receiving occugrid data.",
                       name_.c_str());
    return 1.0;
  }

  // Make sure server is up.
  if (!switching_bound_srv_) {
    ROS_WARN("%s: Switching bound server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    switching_bound_srv_ = nl.serviceClient<value_function::SwitchingTrackingBoundBox>(
      switching_bound_name_.c_str(), true);

    return 1.0;
  }

  // No obstacles. Just check bounds.
  value_function::SwitchingTrackingBoundBox bound;
  bound.request.from_id = incoming_value;
  bound.request.to_id = outgoing_value;
  if (!switching_bound_srv_.call(bound)) {
    ROS_ERROR("%s: Error calling switching bound server.", name_.c_str());
    return 1.0;
  }

  if (position(0) < lower_(0) + bound.response.x ||
      position(0) > upper_(0) - bound.response.x ||
      position(1) < lower_(1) + bound.response.y ||
      position(1) > upper_(1) - bound.response.y ||
      position(2) < lower_(2) + bound.response.z ||
      position(2) > upper_(2) - bound.response.z){
    ROS_WARN_THROTTLE(1.0,"%s: Failed when checking tracking bounds.", name_.c_str());
    return 1.0;
  }

  std::vector<double> min_pos = {position(0) - bound.response.x, 
                                  position(1) - bound.response.y};
  std::vector<double> max_pos = {position(0) + bound.response.x, 
                                  position(1) + bound.response.y};

  std::vector<size_t> min_loc = occu_grids_->RealToSimLoc(min_pos, lower_, upper_);
  std::vector<size_t> max_loc = occu_grids_->RealToSimLoc(max_pos, lower_, upper_);

  // ensure that the coordinates aren't flipped
  if (max_loc[0] < min_loc[0]){
    size_t tmp = min_loc[0];
    min_loc[0] = max_loc[0];
    max_loc[0] = tmp;
  }
  if (max_loc[1] < min_loc[1]){
    size_t tmp = min_loc[1];
    min_loc[1] = max_loc[1];
    max_loc[1] = tmp;
  }

  ROS_ERROR("I'M ABOUT TO INTERPOLATE THE GRID!");
  // 1. interpolate wrt to the given time to get current occupancy grid
  std::vector<double> interpolated_grid = 
    occu_grids_->InterpolateGrid(time, min_loc, max_loc);

  //std::printf("sum of interpolated grid: %5.3f\n", 
  //  std::accumulate(interpolated_grid.begin(), interpolated_grid.end(), 0.0));

  if (interpolated_grid.empty()) {
    ROS_WARN("%s: Failed to interpolate grid -- have not seen any grid msgs.",
             name_.c_str());
    return 1.0;
  }

  double collision_prob = 0.0;

  // TODO need to check that the computation is going in the same
  // order as the quadcopter is positioned
  const size_t height = max_loc[0] - min_loc[0] + 1;
  const size_t width = max_loc[1] - min_loc[1] + 1;

  // 2. find the neighborhood in the occupancy grid where the quadcopter is
  // 3. sum the probabilities inside the neighborhood
  for (size_t x = 0; x < height; x++){
    for (size_t y = 0; y < width; y++){
      const size_t pos = y + width*x;

      if (pos >= interpolated_grid.size())
        ROS_FATAL("Trying to index into interpolated grid with pos out of bounds! pos: %zu", pos);

      //std::cout << "pos: " << pos << "\n";
      //std::cout << "interpolated_grid[pos]: " << interpolated_grid[pos] << "\n";
      //std::cout << "interpolated_grid size: " << interpolated_grid.size() << "\n";
      collision_prob += interpolated_grid[pos];
    }
  }

  ROS_INFO("(x-y dim): (%zu, %zu), total collision_prob: %f", 
    max_loc[0] - min_loc[0], max_loc[1] - min_loc[1], collision_prob);

  //return std::min(collision_prob, 1.0);
  return collision_prob;
}

// Checks for obstacles within a sensing radius. Returns true if at least
// one obstacle was found.
bool SnakesInTesseract::SenseObstacles(const Vector3d& position, double sensor_radius,
                                std::vector<Vector3d>& obstacle_positions,
                                std::vector<double>& obstacle_radii) const {
  // TODO I DONT THINK YOU NEED ME!

  return false;
}

// Checks if a given obstacle is in the environment.
bool SnakesInTesseract::IsObstacle(const Vector3d& obstacle_position,
                            double obstacle_radius) const {
  // TODO I DONT THINK YOU NEED ME!

  return false;
}

// Inherited visualizer from Box needs to be overwritten.
void SnakesInTesseract::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {

  if (pub.getNumSubscribers() <= 0){
		ROS_WARN("Not enough subscribers to Visualize().");
    return;
	}

  // Set up box marker.
  visualization_msgs::Marker cube;
  cube.ns = "cube";
  cube.header.frame_id = frame_id;
  cube.header.stamp = ros::Time::now();
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.color.a = 0.5;
  cube.color.r = 0.3;
  cube.color.g = 0.7;
  cube.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  cube.scale.x = upper_(0) - lower_(0);
  center.x = lower_(0) + 0.5 * cube.scale.x;

  cube.scale.y = upper_(1) - lower_(1);
  center.y = lower_(1) + 0.5 * cube.scale.y;

  cube.scale.z = upper_(2) - lower_(2);
  center.z = lower_(2) + 0.5 * cube.scale.z;

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish cube marker.
  pub.publish(cube);

}

void SnakesInTesseract::VisualizeOccuGrid(const ros::Time now, double fwd_timestep) const{
 	// get the current time and interpolate to get the current grid

  double fwd_time = (now + ros::Duration(fwd_timestep)).toSec();

  std::vector<size_t> min_loc = {0,0};
  std::vector<size_t> max_loc = {occu_grids_->GetHeight(), occu_grids_->GetWidth()};
  std::vector<double> interpolated_grid = 
    occu_grids_->InterpolateGrid(fwd_time, min_loc, max_loc);

	ROS_INFO("Size of interpolated grid %zu", interpolated_grid.size());
	if (!interpolated_grid.empty()){
		// Convert the grid cell probabilities into an array of markers
		// Publish the marker array (with the height of the human boxes)
		for (size_t ii = 0; ii < interpolated_grid.size(); ii++){
			if(interpolated_grid[ii] > threshold_){
				std::cout << "threshold: " << threshold_ << std::endl;
				std::cout << "prob: " << interpolated_grid[ii] << std::endl;
				int row = ii / occu_grids_->GetWidth();
				int col = ii % occu_grids_->GetWidth();

				std::cout << "row: " << row << ", col: " << col << std::endl;

				std::vector<double> real_pos = 
													occu_grids_->SimToRealLoc(row, col, lower_, upper_);

				visualization_msgs::Marker cube;
				cube.ns = "cube";
				cube.header.frame_id = fixed_frame_id_;
				cube.header.stamp = ros::Time::now();
				cube.id = ii+1; // give unique ID to each obstacle
				cube.type = visualization_msgs::Marker::CUBE;
				cube.action = visualization_msgs::Marker::ADD;
				cube.color = ProbToColor(interpolated_grid[ii]);

				geometry_msgs::Point center;

				// Fill in center and scale.
				cube.scale.x = occu_grids_->GetResolution();
				center.x = real_pos[0];

				cube.scale.y = occu_grids_->GetResolution();
				center.y = real_pos[1];

				//TODO: this needs to be multiplied by the human height
				cube.scale.z = interpolated_grid[ii]*1.67; 
				center.z = cube.scale.z/2.0;

				if(cube.scale.z < 1e-8)
					cube.scale.z = 0.001;

				cube.pose.position = center;
				cube.pose.orientation.x = 0.0;
				cube.pose.orientation.y = 0.0;
				cube.pose.orientation.z = 0.0;
				cube.pose.orientation.w = 1.0;

				ROS_INFO("I'm publishing a marker for the occugrid.");
				//arr.markers.push_back(cube);
				occu_grid_marker_pub_.publish(cube);
			}
		}
	}
}


// Converts probability to color message
std_msgs::ColorRGBA SnakesInTesseract::ProbToColor(double probability) const{
  std_msgs::ColorRGBA color;

  color.r = 1.0;
  color.g = 1.0 - probability;
  color.b = probability;
  color.a = 0.8;

	if(probability < 0.001){
		color.r = 0;
		color.g = 0.5;
		color.b = 0.7;
		color.a = 0.6;
	}
  return color;
}

// Add a spherical obstacle of the given radius to the environment.
void SnakesInTesseract::AddObstacle(const Vector3d& point, double r) {
	// TODO DO YOU NEED ME?
}

} //\namespace meta
