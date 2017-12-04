# Represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

std_msgs/Header header

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# Center of the map
geometry_msgs/Pose origin

# The map data, in row-major order, starting with (0,0).
# Stores values between [0,1] that represent likelihood of space being occupied
float64[] data
