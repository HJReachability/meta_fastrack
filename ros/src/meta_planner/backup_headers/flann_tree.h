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
 *          Sylvia Herbert ( sylvia.herbert@berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the FlannTree class, which is a wrapper around the FLANN library's
// fast kdtree index.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_PLANNING_FLANN_TREE_H
#define META_PLANNER_PLANNING_FLANN_TREE_H

#include <meta_planner/planning/waypoint.h>
#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <ros/ros.h>
#include <flann/flann.h>
#include <memory>
#include <vector>
#include <math.h>

namespace meta {
namespace planning {

template<typename S>
class FlannTree : private Uncopyable {
public:
  explicit FlannTree() {}
  ~FlannTree();

  // Insert a new Waypoint into the tree.
  bool Insert(const Waypoint<S>::ConstPtr& waypoint);

  // Nearest neighbor search.
  std::vector<Waypoint<S>::ConstPtr> KnnSearch(S& query, size_t k) const;

  // Radius search.
  std::vector<Waypoint<S>::ConstPtr> RadiusSearch(S& query, double r) const;

private:
  // A Flann kdtree. Searches in this tree return indices, which are then mapped
  // to Waypoint pointers in an array.
  // TODO: fix the distance metric to be something more intelligent.
  std::unique_ptr< flann::KDTreeIndex< flann::L2<double> > > index_;
  std::vector<Waypoint<S>::ConstPtr> registry_;
};

  // -------------------------IMPLEMENTATION------------------------------------
FlannTree::~FlannTree() {
  // Free memory from points in the kdtree.
  if (index_ != nullptr) {
    for (size_t ii = 0; ii < index_->size(); ++ii) {
      double* point = index_->getPoint(ii);
      delete[] point;
    }
  }
}

// Insert a new Waypoint into the tree.
bool FlannTree::Insert(const Waypoint<S>::ConstPtr& waypoint) {
  if (!waypoint.get()) {
    ROS_WARN("FlannTree: Tried to insert a null waypoint.");
    return false;
  }

  // Copy the input point into FLANN's Matrix type.
  const size_t cols = S::StateDimension();
  flann::Matrix<double> flann_point(new double[cols], 1, cols);

  const VectorXd waypoint_vector = waypoint->state.ToVector();

  for (size_t ii = 0; ii < cols; ii++)
    flann_point[0][ii] = waypoint_vector(ii);

  // If this is the first point in the index, create the index and exit.
  if (index_ == nullptr) {
    // Single kd-tree.
    const int kNumTrees = 1;
    index_.reset(new flann::KDTreeIndex< flann::L2<double> >(
      flann_point, flann::KDTreeIndexParams(kNumTrees)));

    index_->buildIndex();
  } else {
    // If the index is already created, add the data point to the index.
    // Rebuild every time the index doubles in size to occasionally rebalance
    // the kdtree.
    const double kRebuildThreshold = 2.0;
    index_->addPoints(flann_point, kRebuildThreshold);
  }

  // Add point to registry.
  registry_.push_back(waypoint);

  return true;
}


// Nearest neighbor search.
template<typename S>
std::vector<Waypoint<S>::ConstPtr>
FlannTree::KnnSearch(S& query, size_t k) const {
  std::vector<Waypoint<S>::ConstPtr> neighbors;

  if (index_ == nullptr) {
    ROS_WARN("FlannTree: Empty index. Add points before querying.");
    return neighbors;
  }

  // Convert the input point to the FLANN format.
  const VectorXd state_vector = query.ToVector();
  const flann::Matrix<double> flann_query(
    state_vector.data(), 1, state_vector.size());

  // Search the kd tree for the nearest neighbor to the query.
  std::vector< std::vector<int> > query_match_indices;
  std::vector< std::vector<double> > query_squared_distances;

  const int num_neighbors_found =
    index_->knnSearch(flann_query, query_match_indices,
                      query_squared_distances, static_cast<int>(k),
                      flann::SearchParams(-1, 0.0, false));

  // Assign output.
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[ query_match_indices[0][ii] ]);

  return neighbors;
}


// Radius search.
template<typename S>
std::vector<Waypoint<S>::ConstPtr
FlannTree::RadiusSearch(S& query, double r) const {
  std::vector<Waypoint<S>::ConstPtr> neighbors;

  if (index_ == nullptr) {
    ROS_WARN("FlannTree: Empty index. Add points before querying.");
    return neighbors;
  }

  // Convert the input point to the FLANN format.
  const VectorXd state_vector = query.ToVector();
  const flann::Matrix<double> flann_query(
    state_vector.data(), 1, state_vector.size());

  // Search the kd tree for the nearest neighbor to the query.
  std::vector< std::vector<int> > query_match_indices;
  std::vector< std::vector<double> > query_squared_distances;

  // FLANN checks Euclidean distance squared, so we pass in r * r.
  int num_neighbors_found =
    index_->radiusSearch(flann_query, query_match_indices,
                         query_squared_distances, r * r,
                         flann::SearchParams(-1, 0.0, false));
  // Assign output.
  for (size_t ii = 0; ii < num_neighbors_found; ii++)
    neighbors.push_back(registry_[ query_match_indices[0][ii] ]);

  return neighbors;
}


} //\namespace planning
} //\namespace meta

#endif
