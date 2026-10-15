std_msgs/Header header
string task_id
int32 class_id
geometry_msgs/PoseStamped observation_tcp
string stage
string outcome
string reason
# HELD, EMPTY, UNKNOWN
string holding
bool place_success
bool return_success
bool stop_confirmed
geometry_msgs/PointStamped precise_center
geometry_msgs/PoseStamped grasp_tcp
float32 association_error_m
