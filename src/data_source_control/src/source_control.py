#!/usr/bin/env python

import math
import rospy                                     
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

# read-only fields
FREQUENCY = 10.0  # refresh frequency of the system
GRAV_ACCELERATION = 9.81
THRESHOLD_ACTIVATE = 10.0  # ratio of lidar acc to imu acc to activate dr
THRESHOLD_DEACTIVATE = 10.0  # ratio of " to " to deactivate dr

# globals
# TODO: We will need a way to transfrom the frames to match orientation
_imu_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
_lidar_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

def update_imu_pv(dead_reckoning_on):
   if dead_reckoning_on:  # approximate current location based on previous vectors
      for val in range(3):
         _imu_pva[0][val] = _imu_pva[0][val] + 
            (1.0/FREQUENCY)*_imu_pva[1][val] + (1.0/FREQUENCY)**2*_imu_pva[2][val]
      for val in range(3):
         _imu_pva[1][val] = _imu_pva[1][val] + (1.0/FREQUENCY)*_imu_pva[2][val]
   else:  # set all the positions and velocities to what is given by the lidar
      for vector in range(2):
         for val in range(3):
            _imu_pva[vector][val] = _lidar_pva[vector][val]

def dead_reckoning(dead_reckoning_active):
   imu_a_mag = math.sqrt(_imu_pva[2][0]**2 + _imu_pva[2][1]**2 + _imu_pva[2][2]**2)
   lidar_a_mag = math.sqrt(_lidar_pva[2][0]**2 + _lidar_pva[2][1]**2 + _lidar_pva[2][2]**2)
   ratio = lidar_a_mag / imu_a_mag
   if dead_reckoning_active:
      if ratio > THRESHOLD_DEACTIVATE:
         return False
   else:
      if ratio > THRESHOLD_ACTIVATE:
         return True

def imu_callback(msg):
   # TODO: transform?
   # adjust for acceleration due to gravity here to simplify operations later
   _imu_pva[2] = [msg.acceleration.x, msg.acceleration.y,
      msg.acceleration.z - GRAV_ACCELERATION]

def lidar_callback(msg):
   # TODO: transform?

def control():
   pub = rospy.Publisher('control_value', Bool)
   sub_imu = rospy.Subscriber('bno005_serial_node', Imu, imu_callback)
   # TODO: we need to decide the type of message(s) sent by the lidar update tool
   # sub_lidar = rospy.Subscriber('lidar_pva_node', , lidar_callback)
   rospy.init_node('source_control', anonymous=True)
   rate = rospy.Rate(int(FREQUENCY))
   dead_reckoning_active = False  # initialize without dead_reckoning
   while not rospy.is_shutdown():
      dead_reckoning_active = dead_reckoning(dead_reckoning_active)
      update_imu_pv(dead_reckoning_active)
      rospy.loginfo(dead_reckoning_active)
      pub.publish(dead_reckoning_active)
      rate.spin()  # unsure if this is needed
      rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterrruptException:
        pass
