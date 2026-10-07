# header is the source RGB header, never the inference completion time.
std_msgs/Header header
# New acquisition creates a new ID; ROI re-verification preserves the ID.
string target_id
string state
string reason
bool valid
# True only when YOLO measured this target on THIS image.
bool measured
builtin_interfaces/Time last_verified_stamp
float32 verification_age_sec
# Pixel coordinates in the original RGB image, NOT ROI/letterbox coordinates.
float32 x
float32 y
float32 width
float32 height
int32 class_id
string class_name
# Last YOLO confidence, not an OpenCV tracker confidence.
float32 confidence
# True only when a fresh mask is published for this header + target_id.
bool mask_available
float32 processing_ms
