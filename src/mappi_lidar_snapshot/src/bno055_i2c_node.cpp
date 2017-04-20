#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "mappi_lidar_snapshot_node");
    ros::NodeHandle handle;

    // Parameters

    ros::spin();

    return 0;
}
