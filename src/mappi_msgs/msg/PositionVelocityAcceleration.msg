
# Stores the position, velocity, and acceleration of one frame relative
# to another.

# This is similar to the geometry_msgs/TransformStamped message, but
# it includes velocity and acceleration and excludes rotation.

std_msgs/Header header

# The name of the child frame
string child_frame_id
# The position of the child frame, meters
geometry_msgs/Vector3 position
# The velocity of the child frame, meters/second
geometry_msgs/Vector3 velocity
# The acceleration of the child frame, meters/second^2
geometry_msgs/Vector3 acceleration
