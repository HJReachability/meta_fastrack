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
// Defines the SubsystemValueFunction class.
//
///////////////////////////////////////////////////////////////////////////////

#include <value_function/subsystem_value_function.h>

namespace meta {

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed.
SubsystemValueFunction::ConstPtr SubsystemValueFunction::
Create(const std::string& file_name) {
  SubsystemValueFunction::ConstPtr ptr(new SubsystemValueFunction(file_name));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
SubsystemValueFunction::SubsystemValueFunction(const std::string& file_name)
  : tracking_bound_(0.0),
    initialized_(Load(file_name)) {}

// Priority of the optimal control at the given state. This is a number
// between 0 and 1, where 1 means the final control signal should be exactly
// the optimal control signal computed by this value function.
double SubsystemValueFunction::Priority(const VectorXd& state) const {
  const double value = Value(state);

  if (value < priority_lower_)
    return 0.0;

  if (value > priority_upper_)
    return 1.0;

  return (value - priority_lower_) / (priority_upper_ - priority_lower_);
}

// Return the voxel index corresponding to the given state.
size_t SubsystemValueFunction::StateToIndex(const VectorXd& punctured) const {
  // Quantize each dimension of the state.
  std::vector<size_t> quantized;
  for (size_t ii = 0; ii < punctured.size(); ii++) {
    if (punctured(ii) < lower_[ii]) {
      ROS_WARN("State is below the SubsystemValueFunction grid in dimension %zu.", ii);
      quantized.push_back(0);
    } else if (punctured(ii) > upper_[ii]) {
      ROS_WARN("State is above the SubsystemValueFunction grid in dimension %zu.", ii);
      quantized.push_back(num_voxels_[ii] - 1);
    } else {
      // In bounds, so quantize. This works because of 0-indexing and casting.
      quantized.push_back(
        static_cast<size_t>((punctured(ii) - lower_[ii]) / voxel_size_[ii]));
    }
  }

  // Convert to row-major order.
  size_t index = quantized[0];

  for (size_t ii = 1; ii < quantized.size(); ii++) {
    index *= num_voxels_[ii];
    index += quantized[ii];
  }

#if 0
  size_t coefficient = 1;

  for (size_t ii = 1; ii <= quantized.size(); ii++) {
    const size_t jj = quantized.size() - ii;

    index += coefficient * quantized[jj];
    coefficient *= num_voxels_[jj];
  }
#endif

  return index;
}

// Compute the grid point below a given state in dimension idx.
double SubsystemValueFunction::
LowerGridPoint(const VectorXd& punctured, size_t idx) const {
#ifdef ENABLE_DEBUG_MESSAGES
  // Catch errors.
  if (punctured.size() != state_dimensions_.size()) {
    ROS_ERROR("Called LowerGridPoint with wrong size vector.");
    return 0.0;
  }

  if (idx >= punctured.size()) {
    ROS_ERROR("Called LowerGridPoint with too large index.");
    return 0.0;
  }
#endif

  // Get center of nearest voxel.
  const double center =
    0.5 * voxel_size_[idx] + lower_[idx] + voxel_size_[idx] *
    std::floor((punctured(idx) - lower_[idx]) / voxel_size_[idx]);

  // Check if center is above us. If so, the lower bound is the voxel below.
  if (center > punctured(idx))
    return center - voxel_size_[idx];

  return center;
}

// Recursive helper function for gradient interpolation.
// Takes a (punctured) state and index along which to interpolate.
VectorXd SubsystemValueFunction::
RecursiveGradientInterpolator(const VectorXd& x, size_t idx) const {
#ifdef ENABLE_DEBUG_MESSAGES
  // Catch errors.
  if (x.size() != state_dimensions_.size()) {
    ROS_ERROR("Called RecursiveGradientInterpolator with wrong size vector.");
    return VectorXd::Zero(state_dimensions_.size());
  }

  if (idx >= x.size()) {
    ROS_ERROR("Called RecursiveGradientInterpolator with too large index.");
    return VectorXd::Zero(state_dimensions_.size());
  }
#endif

  // Assume x's entries prior to idx are equal to the upper/lower bounds of
  // the voxel containing x.

  // Begin by computing the lower and upper bounds of the voxel containing x
  // in dimension idx.
  const double lower = LowerGridPoint(x, idx);
  const double upper = lower + voxel_size_[idx];

  // Compute the fractional distance between lower and upper.
  const double fractional_dist = (x(idx) - lower) / voxel_size_[idx];

  // Split x along dimension idx.
  VectorXd x_lower = x;
  x_lower(idx) = lower;

  VectorXd x_upper = x;
  x_upper(idx) = upper;

  // Base case.
  if (idx == x.size() - 1) {
    return CentralDifference(x_upper) * fractional_dist +
      CentralDifference(x_lower) * (1.0 - fractional_dist);
  }

  // Recursive step.
  return RecursiveGradientInterpolator(x_upper, idx + 1) * fractional_dist +
    RecursiveGradientInterpolator(x_lower, idx + 1) * (1.0 - fractional_dist);
}

// Linearly interpolate to get the value at a particular state.
double SubsystemValueFunction::Value(const VectorXd& state) const {
  const VectorXd punctured = Puncture(state);

  // Get distance from voxel center in each dimension.
  const VectorXd center_distance = DistanceToCenter(punctured);

  // Interpolate.
  const double nn_value = data_[StateToIndex(punctured)];
  double approx_value = nn_value;

  VectorXd neighbor = punctured;
  for (size_t ii = 0; ii < punctured.size(); ii++) {
    // Get neighboring value.
    if (center_distance(ii) >= 0.0)
      neighbor(ii) += voxel_size_[ii];
    else
      neighbor(ii) -= voxel_size_[ii];

    const double neighbor_value = data_[StateToIndex(neighbor)];
    neighbor(ii) = punctured(ii);

    // Compute forward difference.
    const double slope = (center_distance(ii) >= 0.0) ?
      (neighbor_value - nn_value) / voxel_size_[ii] :
      (nn_value - neighbor_value) / voxel_size_[ii];

    // Add to the Taylor approximation.
    approx_value += slope * center_distance(ii);
  }

  return approx_value;
}

// Linearly interpolate to get the gradient at a particular state.
VectorXd SubsystemValueFunction::Gradient(const VectorXd& state) const {
  const VectorXd punctured = Puncture(state);
  const VectorXd gradient = RecursiveGradientInterpolator(punctured, 0);

#if 0
  // Compute gradient at the voxel containing this state.
  const VectorXd nn_gradient = CentralDifference(punctured);
  VectorXd gradient = nn_gradient;

  // Interpolate gradients at each neighbor.
  VectorXd neighbor = punctured;
  for (size_t ii = 0; ii < punctured.size(); ii++) {
    // Get neighboring voxel's gradient.
    if (center_distance(ii) >= 0.0)
      neighbor(ii) += voxel_size_[ii];
    else
      neighbor(ii) -= voxel_size_[ii];

    const VectorXd neighbor_gradient = CentralDifference(neighbor);
    neighbor(ii) = punctured(ii);

    // Compute forward difference.
    const VectorXd diff = (center_distance(ii) >= 0.0) ?
      (neighbor_gradient - nn_gradient) / voxel_size_[ii] :
      (nn_gradient - neighbor_gradient) / voxel_size_[ii];

    // Add to Taylor approximation (separate in each dimension of the gradient).
    gradient += diff * center_distance(ii);
  }
#endif

  return gradient;
}

// Puncture a state vector for the overall system to get a
// valid state vector for this subsystem.
VectorXd SubsystemValueFunction::Puncture(const VectorXd& state) const {
  VectorXd punctured(state_dimensions_.size());

  for (size_t ii = 0; ii < state_dimensions_.size(); ii++)
    punctured(ii) = state(state_dimensions_[ii]);

  return punctured;
}


// Compute a central difference at the voxel containing this state.
VectorXd SubsystemValueFunction::
CentralDifference(const VectorXd& punctured) const {
  VectorXd gradient(punctured.size());

  // Read the gradient one dimension at a time.
  const size_t idx = StateToIndex(punctured);
  for (size_t ii = 0; ii < gradient.size(); ii++)
    gradient(ii) = gradient_[ii][idx];

#if 0
  // Get the value at the voxel containing this state.
  const double nn_value = data_[StateToIndex(punctured)];

  // Compute a central difference in each dimension.
  VectorXd neighbor = punctured;
  for (size_t ii = 0; ii < punctured.size(); ii++) {
    neighbor(ii) += voxel_size_[ii];
    const double forward = data_[StateToIndex(neighbor)];

    neighbor(ii) -= 2.0 * voxel_size_[ii];
    const double backward = data_[StateToIndex(neighbor)];

    neighbor(ii) = punctured(ii);
    gradient(ii) = 0.5 * (forward - backward) / voxel_size_[ii];
  }
#endif

  return gradient;
}

// Compute the distance (vector) from this state to the center
// of the nearest voxel.
VectorXd SubsystemValueFunction::
DistanceToCenter(const VectorXd& punctured) const {
  VectorXd center_distance(punctured.size());
  for (size_t ii = 0; ii < punctured.size(); ii++) {
    const double center =
      std::floor((punctured(ii) - lower_[ii]) / voxel_size_[ii]) * voxel_size_[ii] +
      0.5 * voxel_size_[ii] + lower_[ii];

    center_distance(ii) = punctured(ii) - center;
  }

  return center_distance;
}


// Load from file. Returns whether or not it was successful.
// TODO! Reserve enough space initially for each vector so it does
// resize so much.
bool SubsystemValueFunction::Load(const std::string& file_name) {
  // Open the file.
  mat_t* matfp = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  if (matfp == NULL) {
    ROS_ERROR("Could not open file: %s.", file_name.c_str());
    return false;
  }

  // Read variables from this file.
  const std::string grid_min = "grid_min";
  matvar_t* grid_min_mat = Mat_VarRead(matfp, grid_min.c_str());
  if (grid_min_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_min.c_str());
    return false;
  }

  const std::string grid_max = "grid_max";
  matvar_t* grid_max_mat = Mat_VarRead(matfp, grid_max.c_str());
  if (grid_max_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_max.c_str());
    return false;
  }

  const std::string x_dims = "x_dims";
  matvar_t* x_dims_mat = Mat_VarRead(matfp, x_dims.c_str());
  if (x_dims_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", x_dims.c_str());
    return false;
  }

  const std::string teb = "teb";
  matvar_t* teb_mat = Mat_VarRead(matfp, teb.c_str());
  if (teb_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", teb.c_str());
    return false;
  }

  const std::string priority_lower = "priority_lower";
  matvar_t* priority_lower_mat = Mat_VarRead(matfp, priority_lower.c_str());
  if (priority_lower_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", priority_lower.c_str());
    return false;
  }

  const std::string priority_upper = "priority_upper";
  matvar_t* priority_upper_mat = Mat_VarRead(matfp, priority_upper.c_str());
  if (priority_upper_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", priority_upper.c_str());
    return false;
  }

  const std::string max_planner_speed = "max_planner_speed";
  matvar_t* max_planner_speed_mat =
    Mat_VarRead(matfp, max_planner_speed.c_str());
  if (max_planner_speed_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", max_planner_speed.c_str());
    return false;
  }

  const std::string u_dims = "u_dims";
  matvar_t* u_dims_mat = Mat_VarRead(matfp, u_dims.c_str());
  if (u_dims_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", u_dims.c_str());
    return false;
  }

  const std::string grid_N = "grid_N";
  matvar_t* grid_N_mat = Mat_VarRead(matfp, grid_N.c_str());
  if (grid_N_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_N.c_str());
    return false;
  }

  const std::string data = "data";
  matvar_t* data_mat = Mat_VarRead(matfp, data.c_str());
  if (data_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", data.c_str());
    return false;
  }

  // Populate class variables.
  if (grid_min_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", grid_min.c_str());
    return false;
  }

  size_t num_elements = grid_min_mat->nbytes / grid_min_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    lower_.push_back(static_cast<double*>(grid_min_mat->data)[ii]);
  }

  if (grid_max_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", grid_max.c_str());
    return false;
  }

  num_elements = grid_max_mat->nbytes / grid_max_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    upper_.push_back(static_cast<double*>(grid_max_mat->data)[ii]);
  }

  if (grid_N_mat->data_type != MAT_T_UINT64) {
    ROS_ERROR("%s: Wrong type of data.", grid_N.c_str());
    return false;
  }

  num_elements = grid_N_mat->nbytes / grid_N_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    num_voxels_.push_back(static_cast<size_t*>(grid_N_mat->data)[ii]);
  }

  if (x_dims_mat->data_type != MAT_T_UINT64) {
    ROS_ERROR("%s: Wrong type of data.", x_dims.c_str());
    return false;
  }

  num_elements = x_dims_mat->nbytes / x_dims_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    state_dimensions_.push_back(static_cast<size_t*>(x_dims_mat->data)[ii]);
  }

  if (u_dims_mat->data_type != MAT_T_UINT64) {
    ROS_ERROR("%s: Wrong type of data.", u_dims.c_str());
    return false;
  }

  num_elements = u_dims_mat->nbytes / u_dims_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    control_dimensions_.push_back(static_cast<size_t*>(u_dims_mat->data)[ii]);
  }

  if (teb_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", teb.c_str());
    return false;
  }

  num_elements = teb_mat->nbytes / teb_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    tracking_bound_.push_back(static_cast<double*>(teb_mat->data)[ii]);
  }

  if (priority_lower_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", priority_lower.c_str());
    return false;
  }

  priority_upper_ = *static_cast<double*>(priority_upper_mat->data);

  if (priority_upper_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", priority_upper.c_str());
    return false;
  }

  priority_upper_ = *static_cast<double*>(priority_upper_mat->data);

  if (max_planner_speed_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", max_planner_speed.c_str());
    return false;
  }

  num_elements = max_planner_speed_mat->nbytes / max_planner_speed_mat->data_size;
  if (num_elements != 3)
    ROS_WARN("Number of entries in max planner speed vector was %zu != 3.",
             num_elements);

  for (size_t ii = 0; ii < num_elements; ii++) {
    max_planner_speed_.push_back(static_cast<double*>(max_planner_speed_mat->data)[ii]);
  }

  if (data_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.", data.c_str());
    return false;
  }

  num_elements = data_mat->nbytes / data_mat->data_size;
  for (size_t ii = 0; ii < num_elements; ii++) {
    // NOTE: Could scale up by a large factor here to improve reliability in
    // the sign of the interpolated gradients. Seems to have at best only a
    // minor positive effect on tracking though so reverting to no scaling.
    data_.push_back(static_cast<double*>(data_mat->data)[ii]);
  }

  // Determine voxel size.
  for (size_t ii = 0; ii < num_voxels_.size(); ii++)
    voxel_size_.push_back((upper_[ii] - lower_[ii]) /
                          static_cast<double>(num_voxels_[ii]));

  // Read gradient information one dimension at a time.
  for (size_t ii = 0; ii < num_voxels_.size(); ii++) {
    const std::string deriv = "deriv" + std::to_string(ii);
    matvar_t* deriv_mat = Mat_VarRead(matfp, deriv.c_str());
    if (deriv_mat == NULL) {
      ROS_ERROR("Could not read variable: %s.", deriv.c_str());
      return false;
    }

    num_elements = deriv_mat->nbytes / deriv_mat->data_size;
    if (num_elements != data_.size())
      ROS_ERROR("Derivative %zu had wrong number of elements.", ii);

    std::vector<double> grad;
    for (size_t ii = 0; ii < num_elements; ii++) {
      grad.push_back(static_cast<double*>(deriv_mat->data)[ii]);
    }

    gradient_.push_back(grad);
    Mat_VarFree(deriv_mat);
  }

  // Free memory and close file.
  Mat_VarFree(grid_min_mat);
  Mat_VarFree(grid_max_mat);
  Mat_VarFree(grid_N_mat);
  Mat_VarFree(x_dims_mat);
  Mat_VarFree(u_dims_mat);
  Mat_VarFree(teb_mat);
  Mat_VarFree(priority_lower_mat);
  Mat_VarFree(priority_upper_mat);
  Mat_VarFree(max_planner_speed_mat);
  Mat_VarFree(data_mat);
  Mat_Close(matfp);

  return true;
}

} //\namespace meta
