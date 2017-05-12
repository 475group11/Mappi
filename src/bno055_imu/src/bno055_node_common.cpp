
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include "bno055_node_common.h"

void run_bno055_node(ros::NodeHandle& handle, IMU::Transport& transport) {

    // Get remaining parameters
    // The frame of reference where the information was measured
    std::string frame_id = ros::param::param<std::string>("~frame_id", "base_link");

    // Connect to sensor
    IMU sensor(transport);
    // Advertise
    auto publisher = handle.advertise<sensor_msgs::Imu>("bno055_i2c", 8);

    auto send_callback = [frame_id, publisher, &sensor] (const ros::TimerEvent&) {
        // Send IMU data
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = frame_id;

        imu_data.orientation = sensor.orientation();
        // TODO: This includes gravity. Is this correct?
        imu_data.linear_acceleration = sensor.acceleration();
        imu_data.angular_velocity = sensor.angular_velocity();

        publisher.publish(imu_data);
    };

    ros::Rate loop_rate(100);
    auto timer = handle.createTimer(loop_rate.expectedCycleTime(), send_callback);

    timer.start();
    ros::spin();
}
