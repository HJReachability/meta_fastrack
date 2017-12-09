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
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
SnakesInTesseract::SnakesInTesseract()
  : Box() {}

// Load topics and probability thresholds 
bool SnakesInTesseract::LoadParameters(const ros::NodeHandle& n){

  ros::NodeHandle nl(n);

	std::cout << "In load params" << std::endl;

	// Sensor radius.
  if (!nl.getParam("srv/switching_bound", switching_bound_name_)) return false;

  // Occupancy Grid topic.
  if (!nl.getParam("topics/occupancy_grid_time", occu_grid_topic_)) return false;

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

  return true;
}

void SnakesInTesseract::OccuGridCallback(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg){
	if (occu_grids_ == nullptr){
		// Construct occugrid 
		occu_grids_ = OccuGridTime::Create();
	} 
  // update and convert the incoming message to OccuGridTime data structure
  occu_grids_->FromROSMsg(msg);
}

// Inherited collision checker from Box needs to be overwritten.
// Takes in incoming and outgoing value functions. See planner.h for details.
// TODO: return true/false and also maybe cumulative likelihood of collision?
bool SnakesInTesseract::IsValid(const Vector3d& position,
                         ValueFunctionId incoming_value,
                         ValueFunctionId outgoing_value,
                         double time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!initialized_) {
    ROS_WARN("%s: Tried to collision check an uninitialized SnakesInTesseract.",
             name_.c_str());
    return false;
  }
#endif
	std::cout << "This is the current time: " << time << std::endl;
	// Make sure valid time is being passed in.
	if (time == -1) {
    ROS_WARN("%s: Tried to collision check for NULL time.",
             name_.c_str());
    return true;
  }

  // Make sure server is up.
  if (!switching_bound_srv_) {
    ROS_WARN("%s: Switching bound server disconnected.", name_.c_str());

    ros::NodeHandle nl;
    switching_bound_srv_ = nl.serviceClient<value_function::SwitchingTrackingBoundBox>(
      switching_bound_name_.c_str(), true);

    return false;
  }

  // No obstacles. Just check bounds.
  value_function::SwitchingTrackingBoundBox bound;
  bound.request.from_id = incoming_value;
  bound.request.to_id = outgoing_value;
  if (!switching_bound_srv_.call(bound)) {
    ROS_ERROR("%s: Error calling switching bound server.", name_.c_str());
    return false;
  }

  if (position(0) < lower_(0) + bound.response.x ||
      position(0) > upper_(0) - bound.response.x ||
      position(1) < lower_(1) + bound.response.y ||
      position(1) > upper_(1) - bound.response.y ||
      position(2) < lower_(2) + bound.response.z ||
      position(2) > upper_(2) - bound.response.z)
    return false;

  // TODO: check if interpolation still leads to valid probability distribution

	std::cout << "Interpolating grid...\n";
  // 1. interpolate wrt to the given time to get current occupancy grid
  std::vector<double> interpolated_grid = occu_grids_->InterpolateGrid(time);

	if (interpolated_grid.empty()){
		std::cout << "Failed to interpolate grid -- I probably haven't gotten the first Grid msg!\n";
		return false;
	}

	std::cout << "pos x: " << position(0) << ", response x: " << bound.response.x << std::endl;
  int start_row = (int)(position(0) - bound.response.x); 
  int start_col = (int)(position(1) - bound.response.y);
  int end_row = (int)(position(0) + bound.response.x);
  int end_col = (int)(position(1) + bound.response.y);

	std::cout << start_row << "," << start_col << "," << end_row << "," << end_col << "\n";

  double collision_prob = 0.0;

  // TODO need to debug and check that the computation is going in the same
  // order as the quadcopter is positioned

  // 2. find the neighborhood in the occupancy grid where the quadcopter is 
  // 3. sum the probabilities inside the neighborhood
  for (int x = start_row; x < end_row+1; x++){
    for(int y = start_col; y < end_col+1; y++){
      int pos = y + occu_grids_->GetWidth()*x;
      collision_prob += interpolated_grid[pos];
    }
  }

	std::cout << "collision_prob: " << collision_prob << std::endl;
  // 4. if the summed probability above threshold of collision, return not valid
  if (collision_prob >= threshold_)
    return false;

  return true;
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
	std::cout << "In SnakesInTesseract::Visualize()" << std::endl;
  if (pub.getNumSubscribers() <= 0){
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

  // get the current time and interpolate to get the current grid
  double time = (ros::Time::now()+ros::Duration(1)).toSec();
	std::cout << time << std::endl;
  std::vector<double> interpolated_grid = occu_grids_->InterpolateGrid(time);


	if (!interpolated_grid.empty()){
		//visualization_msgs::MarkerArray arr;

		// Convert the grid cell probabilities into an array of markers
		// Publish the marker array (with the height of the human boxes)
		for (int ii = 0; ii < interpolated_grid.size(); ii++){
			if (interpolated_grid[ii] < threshold_){
				int row = ii / occu_grids_->GetWidth();
				int col = ii % occu_grids_->GetWidth();

				visualization_msgs::Marker cube;
				cube.ns = "cube";
				cube.header.frame_id = frame_id;	
				cube.header.stamp = ros::Time::now();
				cube.id = ii+1; // give unique ID to each obstacle
				cube.type = visualization_msgs::Marker::CUBE;
				cube.action = visualization_msgs::Marker::ADD;
				cube.color = ProbToColor(interpolated_grid[ii]);

				geometry_msgs::Point center;

				// Fill in center and scale.
				cube.scale.x = occu_grids_->GetResolution();
				center.x = row;

				cube.scale.y = occu_grids_->GetResolution();
				center.y = col;

				cube.scale.z = occu_grids_->GetResolution()*5.0; //TODO: this needs to be human height
				center.z = occu_grids_->GetResolution()*2;  

				cube.pose.position = center;
				cube.pose.orientation.x = 0.0;
				cube.pose.orientation.y = 0.0;
				cube.pose.orientation.z = 0.0;
				cube.pose.orientation.w = 1.0;

				//arr.markers.push_back(cube);   
				pub.publish(cube); 
			}
		}

		// Publish occupancy grid markers.
		//pub.publish(arr);
	}
}

// Converts probability to color message
std_msgs::ColorRGBA SnakesInTesseract::ProbToColor(double probability) const{
  std_msgs::ColorRGBA color;

  color.r = 1.0;
  color.b = 0.0;
  color.g = 1.0 - probability;
  color.a = 0.4;

  return color;
}

// Add a spherical obstacle of the given radius to the environment.
void SnakesInTesseract::AddObstacle(const Vector3d& point, double r) {
	// TODO DO YOU NEED ME?
}

} //\namespace meta

