std_msgs/Header header
string task_id
string observation_id
int32 class_id
string instance_id
geometry_msgs/PointStamped coarse_position
# Empty = object. plate = SAM automatic masks + measured tabletop geometry.
string purpose
# Measured support plane in coarse_position.header.frame_id.
float64 support_z
