# Index of waypoint in route.
# Use an index of #waypoints to add a new waypoint at the end.
uint64 index
# New waypoint position for addition or update.
geometry_msgs/Vector3 position
# If true, the waypoint at index will be removed.
bool remove
