#!/usr/bin/env python

import rospy
# Note that while a Bool would be the obvious choice of
# type here, python does not support Bools, and I couldn't
# find anything on ROS answers about how the Bool std_msg
# defines truthiness in standard python types.                                      
from std_msgs.msg import Char
from sensor_msgs.msg import Imu

# globals
# TODO: We will need a way to transfrom the frames to match orientation
imu_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
lidar_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

def update_imu_pv(dead_reckoning_on):
   # this should be called at 10Hz, which gives t = 0.1s
   # approximate via last value of velocity; there might be a better way to do this
   if dead_reckoning_on:
      for val in range(3):
         imu_pva[0][val] = imu_pva[0][val] + 0.1*imu_pva[1][val] + 0.01*imu_pva[2][val]
      for val in range(3):
         imu_pva[1][val] = imu_pva[1][val] + 0.1*imu_pva[2][val]
   else:
      # TODO: constrain PV to LIDAR data

def dead_reckoning():
   # TODO: determine dead_reckoning_on 
   update_imu_pv(dead_reckoning_on)


def imu_callback(msg):
   # TODO: transform?
   # adjust for acceleration due to gravity here to simplify operations later
   imu_pva[2] = [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z - 9.81]

def lidar_callback(msg):
   # TODO: transform?

def control():
   pub = rospy.Publisher('control_value', Char)
   sub_imu = rospy.Subscriber('bno005_serial_node', Imu, imu_callback)
   # TODO: we need to decide the type of message(s) sent by the lidar update tool
   # sub_lidar = rospy.Subscriber('lidar_pva_node', , lidar_callback)
   rospy.init_node('source_control', anonymous=True)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      to_publish = 'n'
      if dead_reckoning():
         to_publish = 'y'
      rospy.loginfo(to_publish)
      pub.publish(to_publish)
      rate.spin()  # unsure if this is needed
      rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterrruptException:
        pass
