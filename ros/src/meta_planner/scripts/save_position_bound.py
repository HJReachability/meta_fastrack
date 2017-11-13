#!/usr/bin/python
"""
Listen for planner/tracker position and TEB, and write to file on quit.

Authors: David Fridovich-Keil ( dfk@eecs.berkeley.edu )
"""

import numpy as np
import rospy

from crazyflie_msgs.msg import PositionStateStamped
from visualization_msgs.msg import Marker

# Set up planner/tracker position.
planner_x = 0.0
planner_y = 0.0
planner_z = 0.0
received_planner = False

tracker_x = 0.0
tracker_y = 0.0
tracker_z = 0.0
received_tracker = False

# Set up bound.
bound_x = 0.0
bound_y = 0.0
bound_z = 0.0
received_bound = False

# Start time.
start_time = -1.0

# Lists for each of the above, recording each at the timer callback.
times = []
planner_xs = []
planner_ys = []
planner_zs = []
tracker_xs = []
tracker_ys = []
tracker_zs = []
bound_xs = []
bound_ys = []
bound_zs = []

# Callback to record planner state.
def PlannerCallback(msg):
    global received_planner
    global planner_x, planner_y, planner_z
    planner_x = msg.state.x
    planner_y = msg.state.y
    planner_z = msg.state.z

    received_planner = True

# Callback to record planner state.
def TrackerCallback(msg):
    global received_tracker
    global tracker_x, tracker_y, tracker_z
    tracker_x = msg.state.x
    tracker_y = msg.state.y
    tracker_z = msg.state.z

    received_tracker = True

# Callback to record tracking bound.
def BoundCallback(msg):
    global received_bound
    global bound_x, bound_y, bound_z
    bound_x = 0.5 * msg.scale.x
    bound_y = 0.5 * msg.scale.y
    bound_z = 0.5 * msg.scale.z

    received_bound = True

# Timer callback. Append last message values to lists.
def TimerCallback(event):
    global start_time
    global planner_x, planner_y, planner_z
    global tracker_x, tracker_y, tracker_z
    global planner_xs, planner_ys, planner_zs
    global tracker_xs, tracker_ys, tracker_zs
    global times

    if (received_planner and received_tracker and received_bound):
        # Catch first time.
        if start_time < 0.0:
            start_time = event.current_real.to_sec()

        # Append to all lists.
        times.append(event.current_real.to_sec() - start_time)
        planner_xs.append(planner_x)
        planner_ys.append(planner_y)
        planner_zs.append(planner_z)
        tracker_xs.append(tracker_x)
        tracker_ys.append(tracker_y)
        tracker_zs.append(tracker_z)
        bound_xs.append(bound_x)
        bound_ys.append(bound_y)
        bound_zs.append(bound_z)

        print "Hit the timer callback."

# Shutdown hook.
def ShutdownHook():
    global planner_xs, planner_ys, planner_zs
    global tracker_xs, tracker_ys, tracker_zs
    global times

    print "Writing to disk before closing..."
    np.savetxt("times.csv", np.array(times), delimiter=",")
    np.savetxt("planner_xs.csv", np.array(planner_xs), delimiter=",")
    np.savetxt("planner_ys.csv", np.array(planner_ys), delimiter=",")
    np.savetxt("planner_zs.csv", np.array(planner_zs), delimiter=",")
    np.savetxt("tracker_xs.csv", np.array(tracker_xs), delimiter=",")
    np.savetxt("tracker_ys.csv", np.array(tracker_ys), delimiter=",")
    np.savetxt("tracker_zs.csv", np.array(tracker_zs), delimiter=",")
    np.savetxt("bound_xs.csv", np.array(bound_xs), delimiter=",")
    np.savetxt("bound_ys.csv", np.array(bound_ys), delimiter=",")
    np.savetxt("bound_zs.csv", np.array(bound_zs), delimiter=",")
        
    print "Done. Bye."




# Main.
if __name__ == "__main__":
    rospy.init_node("position_bound_recorder")
    rospy.Subscriber("/state/position", PositionStateStamped, TrackerCallback)
    rospy.Subscriber("/ref/planner", PositionStateStamped, PlannerCallback)
    rospy.Subscriber("/vis/bound", Marker, BoundCallback)
    rospy.Timer(rospy.Duration(0.05), TimerCallback)

    rospy.on_shutdown(ShutdownHook)

    rospy.spin()


