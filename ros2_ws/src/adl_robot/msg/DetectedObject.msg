# DetectedObject.msg
# Message for detected ArUco objects

# ArUco marker ID
int32 marker_id

# Object type/name (e.g., "book", "water_bottle", "cup")
string object_type

# 3D pose of the object relative to camera frame
geometry_msgs/PoseStamped pose

# Confidence score (0.0 to 1.0)
float32 confidence

# Bounding box in image coordinates
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height

# Timestamp
builtin_interfaces/Time timestamp