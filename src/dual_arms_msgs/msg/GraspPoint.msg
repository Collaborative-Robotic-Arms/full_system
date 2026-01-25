std_msgs/Header header

# The specific ID of the detected brick (matching the YOLO tracking ID)
int32 brick_id

# The calculated grasping pose (Position and Orientation)
geometry_msgs/Pose pose

# The confidence score of the grasp (0.0 to 1.0)
float32 quality
