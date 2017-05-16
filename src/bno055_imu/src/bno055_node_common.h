#ifndef BNO055_NODE_COMMON_H
#define BNO055_NODE_COMMON_H

#include <ros/ros.h>

#include "imu.h"

/**
 * Runs a BNO055 node communicating with the provided transport
 */
void run_bno055_node(ros::NodeHandle& handle, IMU::Transport& transport);

#endif
