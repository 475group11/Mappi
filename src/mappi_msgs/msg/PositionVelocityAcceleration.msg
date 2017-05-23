
# Stores the position, velocity, and acceleration of one frame relative
# to another.

# This is similar to the geometry_msgs/TransformStamped message, but
# it includes velocity and acceleration.

std_msgs/Header header

# The name of the child frame
string child_frame_id
# The position of the child frame
geometry_msgs/Transform transform
# The velocity of the child frame
geometry_msgs/Twist velocity
# The acceleration of the child frame
geometry_msgs/Accel acceleration
