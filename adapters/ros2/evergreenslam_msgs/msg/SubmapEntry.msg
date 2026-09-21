# One submap of the pose graph, at its optimized global pose.
int32 session_id
int32 submap_index
# Scans inserted so far; grows while the submap is open and stops when it is finished. A viewer
# re-queries a submap when this changes.
int32 submap_version
# Global pose of the submap frame in the map frame.
geometry_msgs/Pose pose
bool is_finished
bool is_frozen
