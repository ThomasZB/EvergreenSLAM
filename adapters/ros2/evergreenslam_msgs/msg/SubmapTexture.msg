# One byte per cell: 0..100 occupancy probability in percent, 255 unknown. Row-major with x
# fastest; cell (0, 0) is the corner at slice_pose and y grows with the row index. Same encoding
# as nav_msgs/OccupancyGrid, only unsigned.
uint8[] cells
int32 width
int32 height
float64 resolution
# The cell (0, 0) corner of the grid in the submap frame; yaw is always 0.
geometry_msgs/Pose slice_pose
