# [INTERFACE:CAMERA_FRAME] position is in header.frame_id (camera optical).
# stamp is image acquisition time, not inference completion time.
# Manipulation owns hand-eye/arm TF conversion and TCP goal generation.
std_msgs/Header header
string task_id
string observation_id
# Publish every candidate in this acquisition, with index in [0,count).
# Coordinator waits for the complete batch before nearest-neighbor association.
uint32 candidate_count
uint32 candidate_index
int32 class_id
string instance_id
float32 confidence
# CENTER or SURFACE. CENTER must come from a validated geometric estimator.
string position_kind
geometry_msgs/Point position
float32 position_std_m
# true: use position; false: calculate a SURFACE point using mask + aligned depth
bool has_position
sensor_msgs/Image aligned_depth
sensor_msgs/Image target_mask
sensor_msgs/CameraInfo camera_info
# Banana long-axis yaw in gravity_frame, independently measured at header.stamp
bool axis_valid
float32 axis_yaw
string purpose
# Conservative free placement disk inside an observed plate, metres.
float32 free_radius_m
