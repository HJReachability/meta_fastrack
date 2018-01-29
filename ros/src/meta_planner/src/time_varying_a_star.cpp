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
// Classical A* in 3D, but where collision checks are time-dependent.
// Inherits from the Planner base class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/time_varying_a_star.h>

#include <algorithm>
#include <vector>

namespace meta {

size_t TimeVaryingAStar::Node::num_nodes_ = 0;

// Factory method.
TimeVaryingAStar::Ptr TimeVaryingAStar::
Create(ValueFunctionId incoming_value,
       ValueFunctionId outgoing_value,
       const ProbabilisticBox::ConstPtr& space,
       const Dynamics::ConstPtr& dynamics,
       double grid_resolution,
       double collision_check_resolution) {
  TimeVaryingAStar::Ptr ptr(new TimeVaryingAStar(
    incoming_value, outgoing_value, space, dynamics,
    grid_resolution, collision_check_resolution));
  return ptr;
}

// Derived classes must plan trajectories between two points.
// Budget is the time the planner is allowed to take during planning.
Trajectory::Ptr TimeVaryingAStar::
Plan(const Vector3d& start, const Vector3d& stop,
     double start_time, double budget) const {

	const ros::Time plan_start_time = ros::Time::now();
  const double kStayPutTime = 1.0;

  // Make a set for the open set. This stores the candidate "fringe" nodes 
  // we might want to expand. This is sorted by priority and allows multiple 
  // nodes with the same priority to be in the list. 
  std::multiset<Node::Ptr, typename Node::NodeComparitor> open;

  // Make a hash set for the open set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same 
  // space-time key in log time.
  std::unordered_set<Node::Ptr, 
    typename Node::NodeHasher, typename Node::NodeEqual> open_registry;

  // Make a hash set for the closed set. The key in this hash table
  // is space-time for a node. This is used to find nodes with the same 
  // space-time key in log time.
  std::unordered_set<Node::Ptr, 
    typename Node::NodeHasher, typename Node::NodeEqual> closed_registry;

  // Initialize the priority queue.
  const double start_cost = 0.0;
  const double start_heuristic = ComputeHeuristic(start, stop);
  const Node::Ptr start_node =
    Node::Create(start, nullptr, start_time, start_cost, start_heuristic);

  open.insert(start_node);
  open_registry.insert(start_node);

  // Main loop - repeatedly expand the top priority node and
  // insert neighbors that are not already in the closed list.
  while (true) {
    if (open.size() != open_registry.size()){
      ROS_ERROR("open and open_registry are not the same size!\n open: %zu, open_registry: %zu", 
        open.size(), open_registry.size());
    }

		if ((ros::Time::now() - plan_start_time).toSec() > budget)
			return nullptr;

		if (open.empty()){
			ROS_ERROR_THROTTLE(1.0, "%s: Open list is empty.", name_.c_str());
			return nullptr;
		}

    const Node::Ptr next = *open.begin();

    // TODO this is for debugging!
    next->PrintNode(start_time);
    
    ROS_INFO("%s: Open list size: %zu, Next priority: %f", 
      name_.c_str(), open.size(), next->priority_);

    // Pop the next node from the open_registry and open set.
    open_registry.erase(next); // works because keys in open_registry are unique!    
    RemoveFromMultiset(next, open); 

    // Check if this guy is the goal.
    if (std::abs(next->point_(0) - stop(0)) < grid_resolution_/2.0 &&
				std::abs(next->point_(1) - stop(1)) < grid_resolution_/2.0 &&
				std::abs(next->point_(2) - stop(2)) < grid_resolution_/2.0){
			const Node::ConstPtr parent_node = (next->parent_ == nullptr) ? 
				next : next->parent_;

			// Have to connect the goal point to the last sampled grid point.
      const double best_time = BestPossibleTime(parent_node->point_, stop);
			const double terminus_time = parent_node->time_ + best_time;
      const double terminus_cost = 
        ComputeCostToCome(parent_node, stop, best_time);
      const double terminus_heuristic = 0.0;

			const Node::Ptr terminus = 
				Node::Create(stop, parent_node, terminus_time, terminus_cost, 
                      terminus_heuristic);
      return GenerateTrajectory(terminus);
		}

    // Add this to the closed list.
    closed_registry.insert(next);

    // Expand and add to the list.
    for (const Vector3d& neighbor : Neighbors(next->point_)) {
      // Compute the time at which we'll reach this neighbor.
      const double best_neigh_time = (neighbor.isApprox(next->point_, 1e-8)) ? 
        kStayPutTime : BestPossibleTime(next->point_, neighbor);

      const double neighbor_time = next->time_ + best_neigh_time;
      
      // Compute cost to get to the neighbor.
      const double neighbor_cost = 
        ComputeCostToCome(next, neighbor, best_neigh_time);

      // Compute heuristic from neighbor to stop.
      const double neighbor_heuristic = ComputeHeuristic(neighbor, stop);

      // Discard if this is on the closed list.
      const Node::Ptr neighbor_node =
        Node::Create(neighbor, next, neighbor_time, neighbor_cost, neighbor_heuristic);
      if (closed_registry.count(neighbor_node) > 0)
        continue;

      // Collision check this line segment (and store the collision probability)
      if (!CollisionCheck(next->point_, neighbor, next->time_, 
            neighbor_time, neighbor_node->collision_prob_))
        continue;

      // Check if we're in the open set.
      auto match = open_registry.find(neighbor_node);
      if (match != open_registry.end()) {
        
        std::cout << "I found a match between candidate: \n";
        neighbor_node->PrintNode(start_time);
        std::cout << " and open list node: \n";
        (*match)->PrintNode(start_time);

        // We found a match in the open set.
        if (neighbor_node->priority_ < (*match)->priority_) {
          // Remove the node that matches 
          // and replace it with the new/updated one.
          RemoveFromMultiset(*match, open);
          open.insert(neighbor_node);

          open_registry.erase(*match);
          open_registry.insert(neighbor_node);
        }
      } else {
        // If the neighbor is not in the open set, add him to it.
        open.insert(neighbor_node);
        open_registry.insert(neighbor_node);
      }
    }
  }

}

// Returns the total cost to get to point.
double TimeVaryingAStar::ComputeCostToCome(const Node::ConstPtr& parent, 
  const Vector3d& point, double dt) const{

  if(parent == nullptr){
    ROS_ERROR("Parent shold never be null when computing cost to come!");
    return std::numeric_limits<double>::infinity();
  }

  if (dt < 0.0)
    dt = BestPossibleTime(parent->point_, point);  

  // Cost to get to the parent contains distance + time. Add to this
  // the distance from the parent to the current point and the time

  // option 1: parent->cost_to_come_ + dt
  // option 2 (doesn't work!): parent->cost_to_come_ + dt + 0.001*(parent->point_ - point).norm();
  // option 3: parent->cost_to_come_ + (parent->point_ - point).norm();
  return parent->cost_to_come_ + dt + (parent->point_ - point).norm();
}

// Returns the heuristic for the point.
double TimeVaryingAStar::ComputeHeuristic(const Vector3d& point, 
  const Vector3d& stop) const{

  // This heuristic is the best possible distance + best possible time. 
  // option 1: BestPossibleTime(point, stop)
  // option 2 (doesn't work!): BestPossibleTime(point, stop) + (point - stop).norm()*0.1
  // option 3: (point - stop).norm();
  return (point - stop).norm();
}


// Get the neighbors of the given point on the implicit grid.
// NOTE! Include the given point.
std::vector<Vector3d> TimeVaryingAStar::Neighbors(const Vector3d& point) const {
  std::vector<Vector3d> neighbors;

  // Add all the 27 neighbors (including the point itself).
  for (double x = point(0) - grid_resolution_;
       x < point(0) + grid_resolution_ + 1e-8;
       x += grid_resolution_) {
    for (double y = point(1) - grid_resolution_;
         y < point(1) + grid_resolution_ + 1e-8;
         y += grid_resolution_) {

      // TODO THIS IS A HACK!
      neighbors.push_back(Vector3d(x, y, point(2)));

      //for (double z = point(2) - grid_resolution_;
      //     z < point(2) + grid_resolution_ + 1e-8;
      //     z += grid_resolution_) {
      //  neighbors.push_back(Vector3d(x, y, z));
      //}
    }
  }

  return neighbors;
}

// Collision check a line segment between the two points with the given
// initial start time. Returns true if the path is collision free and
// false otherwise.
bool TimeVaryingAStar::CollisionCheck(const Vector3d& start, const Vector3d& stop,
                                    double start_time, double stop_time, 
                                    double& max_collision_prob) const {

	// Need to check if collision checking against yourself
	const bool same_pt = start.isApprox(stop, 1e-8);

  // Compute the unit vector pointing from start to stop.
  const Vector3d direction = (same_pt) ? Vector3d::Zero() : 
		static_cast<Vector3d>((stop - start) / (stop - start).norm());

  // Compute the dt between query points.
  const double dt = (same_pt) ? (stop_time-start_time)*0.1 : 
		(stop_time - start_time) * collision_check_resolution_ / 
		(stop - start).norm();

  double collision_prob = 0.0;
  // Start at the start point and walk until we get past the stop point.
  Vector3d query(start);
  for (double time = start_time; time < stop_time; time += dt) {
    const bool valid_pt = 
      space_->IsValid(query, incoming_value_, outgoing_value_, collision_prob, time);

    if (collision_prob > max_collision_prob)
      max_collision_prob = collision_prob;

    if (!valid_pt)
      return false;

    // Take a step.
    query += collision_check_resolution_ * direction;
  }

  return true;
}

// This function removes all instances of next with matching
// point and time values from the given multiset (there should only be 1). 
void TimeVaryingAStar::RemoveFromMultiset(const Node::Ptr next, 
  std::multiset<Node::Ptr, typename Node::NodeComparitor>& open) {
  // Get the range of nodes that have equal priority to next
  std::pair<
    std::multiset<Node::Ptr, typename Node::NodeComparitor>::iterator,
    std::multiset<Node::Ptr, typename Node::NodeComparitor>::iterator> matches =
     open.equal_range(next);

  // This guy lets us check the equality of two Nodes.
  Node::NodeEqual equality_checker;

  for (auto it=matches.first; it!=matches.second; ++it) {
    // If the current iterate has the same time and point as next
    // remove the iterate from the open set
    if (equality_checker(*it, next)) {
      open.erase(it);
      return;
    }
  }
}

// Walk backward from the given node to the root to create a Trajectory.
Trajectory::Ptr TimeVaryingAStar::GenerateTrajectory(
  const Node::ConstPtr& node) const {
  // Start with an empty list of positions and times.
  std::vector<Vector3d> positions;
  std::vector<double> times;

  std::cout << "Collision probabilities for generated trajectory:\n";
  // Populate these lists by walking backward, then reverse.
  for (Node::ConstPtr n = node; n != nullptr; n = n->parent_) {
    positions.push_back(n->point_);
    times.push_back(n->time_);
    std::printf("%5.3f, ", n->collision_prob_);
  }
  std::cout << std::endl;

  std::reverse(positions.begin(), positions.end());
  std::reverse(times.begin(), times.end());

  // Lift positions into states.
  const std::vector<VectorXd> states =
    dynamics_->LiftGeometricTrajectory(positions, times);

  // Create dummy list containing value function IDs.
  const std::vector<ValueFunctionId> values(states.size(), incoming_value_);

	ROS_INFO("Returning Trajectory of length %zu.", positions.size());

  // Create a trajectory.
  return Trajectory::Create(times, states, values, values);
}

} //\namespace meta
