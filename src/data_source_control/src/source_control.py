#!/usr/bin/env python

import rospy
# Note that while a Bool would be the obvious choice of
# type here, python does not support Bools, and I couldn't
# find anything on ROS answers about how the Bool std_msg
# defines truthiness in standard python types.                                      
from std_msgs.msg import Char

def dead_reckoning():
   

def control():
   pub = rospy.Publisher('control_value', Char)
   rospy.init_node('source_control', anonymous=True)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      to_publish = 'n'
      if dead_reckoning():
         to_publish = 'y'
      rospy.loginfo(to_publish)
      pub.publish(to_publish)
      rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterrruptException:
        pass
