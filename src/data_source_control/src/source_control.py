#!/usr/bin/env python

import math
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from mappi_msgs.msg import PositionVelocityAcceleration

# read-only fields
FREQUENCY = 10.0  # refresh frequency of the system
GRAV_ACCELERATION = 9.81
THRESHOLD_ACTIVATE = 10.0  # ratio of lidar acc to imu acc to activate dr
THRESHOLD_DEACTIVATE = 10.0  # ratio of " to " to deactivate dr

# globals
# note that _imu_pva position is the transform from odom to dead_reckoning
_imu_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
_lidar_pva = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

def update_imu_pv(dead_reckoning_on):
   if dead_reckoning_on:  # approximate current location based on previous vectors
      for val in range(3):
         _imu_pva[0][val] = _imu_pva[0][val] + 
            (1.0/FREQUENCY)*_imu_pva[1][val] + (1.0/FREQUENCY)**2*_imu_pva[2][val]
      for val in range(3):
         _imu_pva[1][val] = _imu_pva[1][val] + (1.0/FREQUENCY)*_imu_pva[2][val]
   else:  # set all the velocities to what is given by the lidar
      for val in range(2):
         _imu_pva[1][val] = _lidar_pva[1][val]

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
   # TODO: transform IMU from base to map frame
   # adjust for acceleration due to gravity here to simplify operations later
   _imu_pva[2] = [msg.acceleration.x, msg.acceleration.y,
      msg.acceleration.z - GRAV_ACCELERATION]

def lidar_callback(msg):
   _lidar_pva[0] = [msg.position.x, msg.position.y, msg.position.z]
   _lidar_pva[1] = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
   _lidar_pva[2] = [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z]

def control():
   pub = tf2_ros.TransformBroadcaster()
   sub_imu = rospy.Subscriber('bno005_serial_node', Imu, imu_callback)
   sub_lidar = rospy.Subscriber('lidar_pva_node', PositionVelocityAcceleration, lidar_callback)
   rospy.init_node('source_control', anonymous=True)
   rate = rospy.Rate(int(FREQUENCY))
   dead_reckoning_active = False  # initialize without dead_reckoning
   while not rospy.is_shutdown():
      dead_reckoning_active = dead_reckoning(dead_reckoning_active)
      update_imu_pv(dead_reckoning_active)
      rospy.loginfo(dead_reckoning_active)
      t = geometry_msgs.msg.TransformStamped()
      
      t.header.stamp = rospy.Time.now()
      t.header.frame_id = "odom"
      t.child_frame_id = "dead_reckoning"
      t.transform.translation.x = _imu_pva[0][0]
      t.transform.translation.y = _imu_pva[0][1]
      t.transform.translation.z = _imu_pva[0][2]
      # this assumes that other transform values (orientation) default to zero
      
      pub.sendTransform(t)                  
      rate.spin()  # unsure if this is needed
      rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterrruptException:
        pass
