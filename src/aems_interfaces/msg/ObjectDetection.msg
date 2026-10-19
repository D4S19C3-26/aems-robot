# One detector result. header.stamp is the source camera frame timestamp.
std_msgs/Header header
string class_name
float32 confidence
int32[4] bbox_xyxy

# AMCL pose when available, otherwise odometry.
geometry_msgs/PoseStamped robot_pose
