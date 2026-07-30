# Fused object after sensor_fusion_node association + Kalman filtering
int32 fused_id
string class_name
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 confidence
bool in_blind_spot
float32 time_to_collision_s   # -1 if not on a collision path
