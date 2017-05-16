#include <ros/ros.h>

#include <stdexcept>

#include "imu_serial.h"
#include "bno055_node_common.h"

/** Number of times to try connecting */
static const std::size_t CONNECT_TRIES = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "bno055_serial_node");
    ros::NodeHandle handle;

    // Parameters

    // The serial device file
    std::string serial_file = ros::param::param<std::string>("~serial_file", "/dev/serial0");

    for (std::size_t try_number = 0; try_number < CONNECT_TRIES; try_number++) {
        try {
            // Connect to the sensor
            IMUSerial serial(serial_file);
            ROS_INFO("Serial port open");
            run_bno055_node(handle, serial);

            return 0;
        } catch (std::exception& e) {
            ROS_WARN_STREAM("Failed to connect to BNO055: " << e.what());
        }
    }
    ROS_ERROR("Failed to connect to BNO055 after %d tries. Giving up.", CONNECT_TRIES);
    return -37;
}
