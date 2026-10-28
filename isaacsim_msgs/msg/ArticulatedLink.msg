uint8 REVOLUTE=0
uint8 PRISMATIC=1
uint8 CONTINUOUS=2
uint8 FIXED=3

string name
# parent link, empty for the root
string parent
# geometry USD in the model frame, empty for a link without any
string usd_path
# kg, center of mass in the model frame, principal moments along the model axes
float64 mass
geometry_msgs/Point center
geometry_msgs/Vector3 moments
# joint to the parent, unused on the root
string joint
uint8 type
# anchor in the model frame, unit axis
geometry_msgs/Point origin
geometry_msgs/Vector3 axis
float64 lower
float64 upper
# initial joint value
float64 value
JointDynamics dynamics
