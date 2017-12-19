///////////////////////////////////////////////////////////////////////////////
//
// Defines a Box environment with time-varying cube obstacles that represent
// locations that are very likely to have a human at them.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DEMO_SNAKES_IN_TESSERACT_H
#define DEMO_SNAKES_IN_TESSERACT_H

#include <meta_planner/box.h>
#include <utils/types.h>
#include <meta_planner_msgs/OccupancyGridTime.h>
#include <meta_planner_msgs/ProbabilityGrid.h>
#include <demo/occu_grid_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>

#include <vector>

namespace meta {

class SnakesInTesseract : public Box {
public:
  typedef std::shared_ptr<SnakesInTesseract> Ptr;
  typedef std::shared_ptr<const SnakesInTesseract> ConstPtr;

  // Factory method. Use this instead of the constructor.
  static Ptr Create();

  // Destructor.
  ~SnakesInTesseract() {}

  // Inherited collision checker from Box needs to be overwritten.
  // Takes in incoming and outgoing value functions. See planner.h for details.
  bool IsValid(const Vector3d& position,
               ValueFunctionId incoming_value,
               ValueFunctionId outgoing_value,
               double time) const;

  // Check for obstacles within a sensing radius. Returns true if at least
  // one obstacle was sensed.
  bool SenseObstacles(const Vector3d& position, double sensor_radius,
                      std::vector<Vector3d>& obstacle_positions,
                      std::vector<double>& obstacle_radii) const;

  // Check if a given obstacle is in the environment.
  bool IsObstacle(const Vector3d& obstacle_position,
                  double obstacle_radius) const;

  // Inherited visualizer from Box needs to be overwritten.
  void Visualize(const ros::Publisher& pub, const std::string& frame_id) const;
	void VisualizeOccuGrid(const ros::Time now, double fwd_timestep) const;

  // Add a spherical obstacle of the given radius to the environment.
  void AddObstacle(const Vector3d& point, double r);

private:
  explicit SnakesInTesseract();

  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Update the occupancy grid
  void OccuGridCallback(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg);

  // Converts probability to color
  std_msgs::ColorRGBA ProbToColor(double probability) const;

  // List of obstacle locations and radii.
  std::vector<VectorXd> points_;

  // Publisher and subscriber and related topic.
  ros::Subscriber occu_grid_sub_;
	ros::Publisher occu_grid_marker_pub_;
  ros::Publisher trigger_replan_pub_;

  std::string occu_grid_topic_;
	std::string occu_grid_marker_topic_;
  std::string trigger_replan_topic_;

	// Last time we requested new traj
	double last_traj_request_;

  // Frames.
  std::string fixed_frame_id_;

  // Stores time slice and corresponding occupancy grid at time slice
  OccuGridTime::Ptr occu_grids_;

  // Probability threshold for isValid() returning true. Is value in [0,100]
  // If probability of collision is greater than threshold_ then false
  double threshold_;
};

} //\namespace meta

#endif
