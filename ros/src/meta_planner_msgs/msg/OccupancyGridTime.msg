# This represents a sequence of 2-D grid maps, where each OccupancyGrid is
# generated for a certain timestep (in the future). Each grid cell represents
# the probability of occupancy

# Ordered array of OccupancyGrids for some number of timesteps in the future. 
# Each OccupancyGrid has a Header whose time stamp component records what 
# time interval this grid was generated for. 
# gridarray = [OccupancyGrid time1, OccupancyGrid time2, OccupancyGrid time3...]
OccupancyGrid[] gridarray
