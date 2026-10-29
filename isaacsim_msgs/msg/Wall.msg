string name
geometry_msgs/Point start
geometry_msgs/Point end
Material material
float64 thickness 0.1
# drawn for cameras, false keeps lidar, depth and collision
bool visible true
# has collision
bool solid true
# casts shadows
bool shadows true
# world-frame m/s of a kinematic box, read when is_static is false
geometry_msgs/Vector3 velocity
# false spawns a kinematic box carried at velocity
bool is_static true
