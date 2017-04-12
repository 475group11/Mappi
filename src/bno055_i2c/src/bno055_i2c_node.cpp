#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include <iostream>
#include <stdexcept>
#include <memory>

#include "imu.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bno055_i2c");
    ros::NodeHandle handle;

    // Parameters

    // The I2C device file
    std::string i2c_file = "/dev/i2c-1";
    ros::param::get("~i2c_file", i2c_file);
    // The address of the sensor
    int sensor_address = 0x28;
    ros::param::get("~sensor_address", sensor_address);
    // The frame of reference where the information was measured
    std::string frame_id = "base_link";
    ros::param::get("~frame_id", frame_id);

    // Check sensor address
    if ((sensor_address & ~0xFF) != 0) {
        ROS_ERROR("Sensor address greater than 255");
        return -28;
    }

    // Connect to the sensor
    std::unique_ptr<IMU> sensor;
    try {
        sensor.reset(new IMU(i2c_file, std::uint8_t(sensor_address)));
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Failed to connect to BNO055: " << e.what());
        return -29;
    }
    // Advertise
    auto publisher = handle.advertise<sensor_msgs::Imu>("bno055_i2c", 8);

    auto send_callback = [frame_id, publisher, &sensor] (const ros::TimerEvent&) {
        ROS_DEBUG("Running!");

        // Send IMU data
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = frame_id;

        imu_data.orientation = sensor->orientation();
        // TODO: This includes gravity. Is this correct?
        imu_data.linear_acceleration = sensor->acceleration();
        imu_data.angular_velocity = sensor->angular_velocity();

        publisher.publish(imu_data);
    };

    ros::Rate loop_rate(1);
    auto timer = handle.createTimer(loop_rate.expectedCycleTime(), send_callback);

    timer.start();
    ros::spin();

    return 0;
}
