#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <bno055_msgs/Calibration.h>

#include "bno055_node_common.h"

void run_bno055_node(ros::NodeHandle& handle, IMU::Transport& transport) {

    // Get remaining parameters
    // The frame of reference where the information was measured
    std::string frame_id = ros::param::param<std::string>("~frame_id", "base_link");

    // Connect to sensor
    IMU sensor(transport);
    ROS_INFO("IMU connected");
    ROS_INFO("Software revision %x", sensor.software_revision());
    ROS_INFO("Bootloader revision %x", sensor.bootloader_revision());
    ROS_INFO_STREAM("System status " << sensor.status());
    ROS_INFO_STREAM("Error flag " << sensor.error());

    // Advertise
    auto publisher = handle.advertise<sensor_msgs::Imu>("imu", 8);
    auto calibration_publisher = handle.advertise<bno055_msgs::Calibration>("imu_calibration", 8);

    // The last message that was sent
    sensor_msgs::Imu previous_message;

    auto send_callback = [frame_id, publisher, calibration_publisher, &sensor, &previous_message]
        (const ros::TimerEvent&) {
        try {
	    // Send IMU data
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = frame_id;

            imu_data.orientation = sensor.orientation();
            // This includes gravity. Assumed correct.
            imu_data.linear_acceleration = sensor.acceleration();
            imu_data.angular_velocity = sensor.angular_velocity();

            publisher.publish(imu_data);
            previous_message = std::move(imu_data);
        } catch (std::exception& e) {
            ROS_WARN("Failed to read data from sensor: %s", e.what());
            publisher.publish(previous_message);
        }
        try {
            // Send calibration
            const auto imu_calibration = sensor.calibration_state();
            bno055_msgs::Calibration calibration_message;
            calibration_message.accelerometer_status = static_cast<std::uint8_t>(imu_calibration.accelerometer);
            calibration_message.gyroscope_status = static_cast<std::uint8_t>(imu_calibration.gyroscope);
            calibration_message.magnetometer_status = static_cast<std::uint8_t>(imu_calibration.magnetometer);
            calibration_message.system_status = static_cast<std::uint8_t>(imu_calibration.system);

            calibration_publisher.publish(calibration_message);
        } catch (std::exception& e) {
            ROS_WARN("Failed to read calibration status from sensor: %s", e.what());
        }
    };

    ros::Rate loop_rate(50);
    auto timer = handle.createTimer(loop_rate.expectedCycleTime(), send_callback);

    timer.start();
    ros::spin();
}
