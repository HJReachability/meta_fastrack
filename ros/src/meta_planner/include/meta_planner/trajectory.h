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
// Defines the Trajectory struct.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_TRAJECTORY_H
#define META_PLANNER_TRAJECTORY_H

#include <value_function/dynamics.h>
#include <utils/types.h>
#include <utils/message_interfacing.h>

#include <meta_planner_msgs/Trajectory.h>
#include <meta_planner_msgs/State.h>

#include <value_function_srvs/GeometricPlannerTime.h>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <utility>
#include <map>
#include <string>
#include <iostream>
#include <exception>
#include <memory>

namespace meta {

class Trajectory {
public:
  typedef std::shared_ptr<Trajectory> Ptr;
  typedef std::shared_ptr<const Trajectory> ConstPtr;

  // Factory method. Use this instead of the constructor.
  static inline Ptr Create() {
    Ptr ptr(new Trajectory());
    return ptr;
  }

  // Factory constructor from times, states, values.
  static Ptr Create(const std::vector<double>& times,
                    const std::vector<VectorXd>& states,
                    const std::vector<ValueFunctionId>& control_values,
                    const std::vector<ValueFunctionId>& bound_values);

  // Factory constructor from ROS message and an ordered list of all
  // possible ValueFunctions.
  static Ptr Create(const meta_planner_msgs::Trajectory::ConstPtr& msg);

  // Factory constructor to create a Trajectory as the remainder of the
  // given Trajectory after the specified time point.
  static Ptr Create(const ConstPtr& other, double start);

  // Clear out this Trajectory.
  void Clear();

  // Add a (state, time) tuple to this Trajectory.
  void Add(double time,
           const VectorXd& state,
           ValueFunctionId control_value,
           ValueFunctionId bound_value);

  // Add a whole other Trajectory to this one.
  void Add(const ConstPtr& other);

  // Check if this trajectory is empty.
  bool IsEmpty() const;

  // Number of waypoints.
  size_t Size() const;

  // Total time length of the trajectory.
  double Time() const;

  // Swap out the control value function in this trajectory and update time
  // stamps accordingly.
  void ExecuteSwitch(ValueFunctionId value, ros::ServiceClient& best_time_srv);

  // Adjust the time stamps for this trajectory to start at the given time.
  void ResetStartTime(double start);

  // Accessors.
  const VectorXd& LastState() const;
  const VectorXd& FirstState() const;
  double LastTime() const;
  double FirstTime() const;
  ValueFunctionId LastControlValueFunction() const;
  ValueFunctionId FirstControlValueFunction() const;
  ValueFunctionId LastBoundValueFunction() const;
  ValueFunctionId FirstBoundValueFunction() const;

  // Find the state corresponding to a particular time via linear interpolation.
  VectorXd GetState(double time) const;

  // Return the ID of the value function being used at this time.
  ValueFunctionId GetControlValueFunction(double time) const;
  ValueFunctionId GetBoundValueFunction(double time) const;

  // Convert to ROS message.
  meta_planner_msgs::Trajectory ToRosMessage() const;

  // Visualize this trajectory in RVIZ.
  void Visualize(const ros::Publisher& pub,
                 const std::string& frame_id) const;

  // Print this trajectory to stdout.
  void Print(const std::string& prefix) const;

private:
  Trajectory() {}

  // Compute the color (on a red-blue colormap) at a particular time.
  std_msgs::ColorRGBA Colormap(double time) const;

  // Private struct to hold a state and a value function.
  struct StateValue {
    VectorXd state_;
    ValueFunctionId control_value_;
    ValueFunctionId bound_value_;

    StateValue(const VectorXd& state,
               ValueFunctionId control_value,
               ValueFunctionId bound_value)
      : state_(state),
        control_value_(control_value),
        bound_value_(bound_value) {}
    ~StateValue() {}
  };

  // Map from time stamp to corresponding state/value function.
  std::map<double, StateValue> map_;
};

// ---------------------- IMPLEMENT INLINE FUNCTIONS ------------------------ //

// Clear out this Trajectory.
inline void Trajectory::Clear() {
  map_.clear();
}

// Add a (state, time) tuple to this Trajectory.
inline void Trajectory::Add(double time,
                            const VectorXd& state,
                            ValueFunctionId control_value,
                            ValueFunctionId bound_value) {
  map_.insert({ time, StateValue(state, control_value, bound_value) });
}

// Add a whole other Trajectory to this one.
inline void Trajectory::Add(const ConstPtr& other) {
  map_.insert(other->map_.begin(), other->map_.end());
}

// Check if this trajectory is empty.
inline bool Trajectory::IsEmpty() const {
  return map_.empty();
}

// Number of waypoints.
inline size_t Trajectory::Size() const {
  return map_.size();
}

// Total time length of the trajectory.
inline double Trajectory::Time() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get time length of empty trajectory.");
    return 0.0;
  }
#endif

  return LastTime() - FirstTime();
}

// Accessors.
inline const VectorXd& Trajectory::LastState() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last state of empty trajectory.");
    throw std::underflow_error("Attempted last state of empty trajectory.");
  }
#endif

  return (--map_.end())->second.state_;
}

inline const VectorXd& Trajectory::FirstState() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get first state of empty trajectory.");
    throw std::underflow_error("Attempted first state of empty trajectory.");
  }
#endif

  return map_.begin()->second.state_;
}

inline double Trajectory::LastTime() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last time of empty trajectory.");
    throw std::underflow_error("Attempted last time of empty trajectory.");
  }
#endif

  return (--map_.end())->first;
}

inline double Trajectory::FirstTime() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get first time of empty trajectory.");
    throw std::underflow_error("Attempted first time of empty trajectory.");
  }
#endif

  return map_.begin()->first;
}

inline ValueFunctionId Trajectory::LastControlValueFunction() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last ValueFunction of empty trajectory.");
    throw std::underflow_error("Attempted last ValueFunction of empty trajectory.");
  }
#endif

  return (--map_.end())->second.control_value_;
}

inline ValueFunctionId Trajectory::FirstControlValueFunction() const {
#ifdef ENABLE_DEBUG_MESSAGES
    if (IsEmpty()) {
      ROS_WARN("Tried to get first ValueFunction of empty trajectory.");
      throw std::underflow_error("Attempted first ValueFunction of empty trajectory.");
    }
#endif

    return map_.begin()->second.control_value_;
  }

inline ValueFunctionId Trajectory::LastBoundValueFunction() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last ValueFunction of empty trajectory.");
    throw std::underflow_error("Attempted last ValueFunction of empty trajectory.");
  }
#endif

  return (--map_.end())->second.bound_value_;
}

inline ValueFunctionId Trajectory::FirstBoundValueFunction() const {
#ifdef ENABLE_DEBUG_MESSAGES
    if (IsEmpty()) {
      ROS_WARN("Tried to get first ValueFunction of empty trajectory.");
      throw std::underflow_error("Attempted first ValueFunction of empty trajectory.");
    }
#endif

    return map_.begin()->second.bound_value_;
  }

} //\namespace meta

#endif
