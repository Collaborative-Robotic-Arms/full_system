# msg/Brick.msg
std_msgs/Header header
# Unique ID
int32 id

# Enumerated Types (Constants)
uint8 I_BRICK=0
uint8 L_BRICK=1
uint8 T_BRICK=2
uint8 Z_BRICK=3

# The actual field to store the type (holds one of the values above)
uint8 type


# Enumerated Sides
uint8 ABB=0
uint8 AR4=1
uint8 GRID=2

# The actual field to store the side (holds one of the values above)
uint8 side

# The Pose (Position and Orientation)
geometry_msgs/Pose pose