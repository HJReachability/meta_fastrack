#!/usr/bin/python
"""
Plot subsystems (x, vx), (y, vy), (z, vz) by listening to the PositionStateEstimator.

Authors: David Fridovich-Keil ( dfk@eecs.berkeley.edu )
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import rospy
from crazyflie_msgs.msg import PositionStateStamped

#matplotlib.rcParams['text.usetex'] = True

# Set up planner reference point in global scope.
planner_x = 0.0
planner_y = 0.0
planner_z = 0.0

received_reference = False

# Set up plots in global scope.
plt.ion()
fig, ax = plt.subplots(3)
plots = []

# X-subsystem.
plots.append(ax[0].scatter([], []))
ax[0].set_title("X subsystem")
ax[0].set_xlabel("V_{x} (m/s)")
ax[0].set_ylabel("x (m)")

#ax[0].set_xlabel(r"$V_{x}$ (m/s)")
#ax[0].set_ylabel(r"$x$ (m)")
ax[0].set_xlim(-2, 2)
ax[0].set_ylim(-2, 2)

# Y-subsystem.
plots.append(ax[1].scatter([], []))
ax[1].set_title("Y subsystem")
ax[1].set_xlabel("V_{y} (m/s)")
ax[1].set_ylabel("y (m)")

#ax[1].set_xlabel(r"$V_{y}$ (m/s)")
#ax[1].set_ylabel(r"$y$ (m)")
ax[1].set_xlim(-2, 2)
ax[1].set_ylim(-2, 2)

# Z-subsystem.
plots.append(ax[2].scatter([], []))
ax[2].set_title("Z subsystem")
ax[2].set_xlabel("V_{z} (m/s)")
ax[2].set_ylabel("z (m)")

#ax[2].set_xlabel(r"$V_{z}$ (m/s)")
#ax[2].set_ylabel(r"$z$ (m)")
ax[2].set_xlim(-2, 2)
ax[2].set_ylim(-2, 2)

# Callback to plot a new state in each of the three subsystem plots.
def StateCallback(msg):
    global planner_x, planner_y, planner_z
    global points, ax
    global received_reference

    if not received_reference:
        return

    MAX_POINTS = 5000

    # Unpack msg.
    x = msg.state.x - planner_x
    vx = msg.state.x_dot
    y = msg.state.y - planner_y
    vy = msg.state.y_dot
    z = msg.state.z - planner_z
    vz = msg.state.z_dot

    # X subsystem.
    points = plots[0].get_offsets()
    points = np.append(points, np.array([vx, x]))

    if len(points) > MAX_POINTS:
        points = points[-MAX_POINTS:]

    plots[0].set_offsets(points)
    ax[0].set_xlim(points[:, 0].min() - 0.1, points[:, 0].max() + 0.1)
    ax[0].set_ylim(points[:, 1].min() - 0.1, points[:, 1].max() + 0.1)

    # Y subsystem.
    points = plots[1].get_offsets()
    points = np.append(points, np.array([vy, y]))

    if len(points) > MAX_POINTS:
        points = points[-MAX_POINTS:]

    plots[1].set_offsets(points)
    ax[1].set_xlim(points[:, 0].min() - 0.1, points[:, 0].max() + 0.1)
    ax[1].set_ylim(points[:, 1].min() - 0.1, points[:, 1].max() + 0.1)

    # Z subsystem.
    points = plots[2].get_offsets()
    points = np.append(points, np.array([vz, z]))

    if len(points) > MAX_POINTS:
        points = points[-MAX_POINTS:]

    plots[2].set_offsets(points)
    ax[2].set_xlim(points[:, 0].min() - 0.1, points[:, 0].max() + 0.1)
    ax[2].set_ylim(points[:, 1].min() - 0.1, points[:, 1].max() + 0.1)

# Callback to update the planner state.
def ReferenceCallback(msg):
    global planner_x, planner_y, planner_z
    global received_reference

    planner_x = msg.state.x
    planner_y = msg.state.y
    planner_z = msg.state.z

    received_reference = True

# Main.
if __name__ == "__main__":
    rospy.init_node("state_plotter")
    rospy.Subscriber("/state/position", PositionStateStamped, StateCallback)
    rospy.Subscriber("/ref/planner", PositionStateStamped, ReferenceCallback)

    while not rospy.is_shutdown():
        # Update the figure.
        fig.canvas.draw()

        # Go to sleep.
        rospy.sleep(1.0)
