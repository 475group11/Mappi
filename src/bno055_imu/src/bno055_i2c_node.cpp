#include <ros/ros.h>

#include <stdexcept>

#include "imu_i2c.h"
#include "bno055_node_common.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bno055_i2c_node");
    ros::NodeHandle handle;

    // Parameters

    // The I2C device file
    std::string i2c_file = ros::param::param<std::string>("~i2c_file", "/dev/i2c-1");
    // The address of the sensor
    int sensor_address = ros::param::param<int>("~sensor_address", 0x28);

    // Check sensor address
    if ((sensor_address & ~0xFF) != 0) {
        ROS_ERROR("Sensor address must not be greater than 255");
        return -28;
    }

    try {
        // Connect to the sensor
        IMUI2C i2c(i2c_file, std::uint8_t(sensor_address));
        run_bno055_node(handle, i2c);

        return 0;
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Failed to connect to BNO055: " << e.what());
        return -29;
    }
}
