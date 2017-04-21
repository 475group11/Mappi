#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mappi_msgs/Snapshot.h>

typedef boost::function<bool (mappi_msgs::Snapshot::Request&, mappi_msgs::Snapshot::Response&)> handler_callback;

int main(int argc, char** argv) {
    ros::init(argc, argv, "mappi_lidar_snapshot_node");
    ros::NodeHandle handle;

    ROS_INFO("mappi_lidar_snapshot_node starting up\n");

    auto take_snapshot = [] (mappi_msgs::Snapshot::Request&, mappi_msgs::Snapshot::Response& response) {
        ROS_INFO("Taking snapshot");
        response.result = 0;
        return true;
    };

    auto service = handle.advertiseService("snapshot", handler_callback(take_snapshot));

    ros::spin();

    return 0;
}
