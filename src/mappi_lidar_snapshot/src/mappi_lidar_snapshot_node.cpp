#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mappi_lidar_snapshot_node");
    ros::NodeHandle handle;

    std::cout << "mappi_lidar_snapshot_node starting up\n";

    // Parameters

    ros::spin();

    return 0;
}
