string usd_path
string name
geometry_msgs/Pose pose
Scale scale
# world-frame m/s of a moving box, read by EditPrims velocity
geometry_msgs/Vector3 velocity
# false asks for a kinematic mover, SpawnPrims refuses it for USD models
bool is_static true