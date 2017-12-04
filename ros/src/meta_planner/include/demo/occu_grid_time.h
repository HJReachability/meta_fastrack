#ifndef OCCU_GRID_TIME_H
#define OCCU_GRID_TIME_H

//#include <meta_planner/types.h>
#include <meta_planner_msgs/OccupancyGridTime.h>
#include <meta_planner_msgs/ProbabilityGrid.h>

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

  // converts a given OccupancyGridTime msg to internal OccuGridTime data struct
  void FromROSMsg(const meta_planner_msgs::OccupancyGridTime::ConstPtr& msg);

  // given a time, returns an interpolated flattened 1D occupancy grid
  std::vector<double> InterpolateGrid(double curr_time);

  int GetWidth() const;
  int GetHeight() const;
  double GetResolution() const;

private:

  OccuGridTime();

  // dimensions of each of the grids
  int height_;
  int width_;
  double resolution_;
  std::vector<double> origin_;

  // stores list of "flattened" 1D occupancy grids
  std::vector<std::vector<double> > grids_;
  // stores corresponding times for each of the grids
  std::vector<double> times_;

};

} //\namespace meta

#endif

