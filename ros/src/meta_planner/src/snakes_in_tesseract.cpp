///////////////////////////////////////////////////////////////////////////////
//
// Defines a Box environment with time-varying cube obstacles that represent 
// locations that are very likely to have a human at them. 
//
///////////////////////////////////////////////////////////////////////////////

#include <demo/snakes_in_tesseract.h>

namespace meta {

// Factory method. Use this instead of the constructor.
SnakesInTesseract::Ptr SnakesInTesseract::Create(double prob_thresh) {
  SnakesInTesseract::Ptr ptr(new SnakesInTesseract());
  ptr->threshold_ = prob_thresh;
  ptr->occu_grid_topic_ = "occupancy_grid_time";
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
SnakesInTesseract::SnakesInTesseract()
  : Box() {}

// Initialize this class with all parameters and callbacks.
bool SnakesInTesseract::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "snakes_in_tesseract");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

// Load all parameters from config files.
bool SnakesInTesseract::LoadParameters(const ros::NodeHandle& n) {
  //TODO: do you need me?
  return true;
}

// Register all callbacks and publishers.
bool SnakesInTesseract::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriber.
  occu_grid_sub_ = nl.subscribe(
    occu_grid_topic_.c_str(), 1, &SnakesInTesseract::OccuGridCallback, this);

  return true;
}

void SnakesInTesseract::OccuGridCallback(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg){

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
    ROS_WARN("%s: Tried to collision check an uninitialized BallsInBox.",
             name_.c_str());
    return false;
  }
#endif

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
  // 1. interpolate wrt to the given time to get current occupancy grid
  std::vector<double> interpolated_grid = occu_grids_->InterpolateGrid(time);

  int start_row = (int)lower_(0); 
  int start_col = (int)lower_(1);
  int end_row = (int)upper_(0);
  int end_col = (int)upper_(1);

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
  // TODO IMPLEMENT ME!
  obstacle_positions.clear();
  /*obstacle_radii.clear();

  for (size_t ii = 0; ii < points_.size(); ii++){
    if ((position - points_[ii]).norm() <= radii_[ii] + sensor_radius) {
      obstacle_positions.push_back(points_[ii]);
      obstacle_radii.push_back(radii_[ii]);
    }
  }*/

  return obstacle_positions.size() > 0;
}

// Checks if a given obstacle is in the environment.
bool SnakesInTesseract::IsObstacle(const Vector3d& obstacle_position,
                            double obstacle_radius) const {
  // TODO IMPLEMENT ME!
  /*
  for (size_t ii = 0; ii < points_.size(); ii++)
    if ((obstacle_position - points_[ii]).norm() < 1e-8 &&
        std::abs(obstacle_radius - radii_[ii]) < 1e-8)
      return true;
  */
  return false;
}


// Inherited visualizer from Box needs to be overwritten.
void SnakesInTesseract::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {

  if (pub.getNumSubscribers() <= 0)
    return;

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


  // get the current time
  // Interpolate the OccupancyGridTime to get the current grid
  double time = ros::Time::now().toSec();
  std::vector<double> interpolated_grid = occu_grids_->InterpolateGrid(time);

  // Convert the grid cell probabilities into an array of markers
  // Publish the marker array (with the height of the human boxes)

  visualization_msgs::MarkerArray arr;

  for (int ii = 0; ii < interpolated_grid.size(); ii++){
    int row = ii / occu_grids_->GetWidth();
    int col = ii % occu_grids_->GetWidth();

    visualization_msgs::Marker cube;
    cube.ns = "cube";
    cube.header.frame_id = frame_id;
    cube.header.stamp = ros::Time::now();
    cube.id = 0;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.action = visualization_msgs::Marker::ADD;
    cube.color = ProbToColor(interpolated_grid[ii]);

    geometry_msgs::Point center;

    // Fill in center and scale.
    cube.scale.x = occu_grids_->GetResolution();
    center.x = row;

    cube.scale.y = occu_grids_->GetResolution();
    center.y = col;

    cube.scale.z = occu_grids_->GetResolution()*2.0; //TODO: this needs to be human height
    center.z = 0.0;  

    cube.pose.position = center;
    cube.pose.orientation.x = 0.0;
    cube.pose.orientation.y = 0.0;
    cube.pose.orientation.z = 0.0;
    cube.pose.orientation.w = 1.0;

    arr.markers.push_back(cube);    
  }

  // Publish occupancy grid markers.
  pub.publish(arr);
}

// Converts probability to color message
std_msgs::ColorRGBA SnakesInTesseract::ProbToColor(double probability){
  std_msgs::ColorRGBA color;

  color.r = 1.0;
  color.b = 1.0 - probability;
  color.g = 0.0;
  color.a = 0.5;

  return color;
}

// Add a spherical obstacle of the given radius to the environment.
void SnakesInTesseract::AddObstacle(const Vector3d& point, double r) {
  const double kSmallNumber = 1e-8;

#ifdef ENABLE_DEBUG_MESSAGES
  if (r < kSmallNumber)
    ROS_ERROR("Radius was too small: %f.", r);
#endif

  points_.push_back(point);
  //radii_.push_back(std::max(r, kSmallNumber));
}

} //\namespace meta

int main(int argc, char** argv) {
  std::cout << "hellooo!" << std::endl;
}
