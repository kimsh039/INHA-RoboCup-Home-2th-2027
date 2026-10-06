# Horizontal support surface (table / shelf top) seen by the Mid-360, in the array's frame.
# Stable id while the surface stays in view; the outline is the observed extent.
uint32 id
# Mean surface z in the frame, and the same as height above the floor.
float32 z
float32 height
# Rectangle fitted to the observed top: centre, long-side direction (theta), size.
geometry_msgs/Pose2D centre
float32 length
float32 width
# Corners counter-clockwise; side i runs corners[i] -> corners[i+1].
geometry_msgs/Point[4] corners
# Fraction of each side's length with lidar returns next to it (0 = not seen / occluded).
float32[4] edge_observed
uint32 inliers
# Standard deviation of the surface points' z, m.
float32 residual
