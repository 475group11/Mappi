#include <ros/ros.h>

#include <stdexcept>

#include "imu_serial.h"
#include "bno055_node_common.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bno055_serial_node");
    ros::NodeHandle handle;

    // Parameters

    // The serial device file
    std::string serial_file = ros::param::param<std::string>("~serial_file", "/dev/serial0");

    try {
        // Connect to the sensor
        IMUSerial serial(serial_file);
        run_bno055_node(handle, serial);

        return 0;
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Failed to connect to BNO055: " << e.what());
        return -29;
    }
}
