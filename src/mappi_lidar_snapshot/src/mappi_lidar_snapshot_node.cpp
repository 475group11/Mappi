#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mappi_msgs/Snapshot.h>

typedef boost::function<bool (mappi_msgs::Snapshot::Request&, mappi_msgs::Snapshot::Response&)> handler_callback_t;
typedef boost::function<void (const sensor_msgs::LaserScan::ConstPtr&)> laser_callback_t;

int main(int argc, char** argv) {
    ros::init(argc, argv, "mappi_lidar_snapshot_node");
    ros::NodeHandle handle;

    // Parameters

    // The topic to subscribe to for laser scan messages
    std::string laser_topic = ros::param::param<std::string>("laser_topic", "/laser");
    // The maximum age for a scan message before it is considered stale,
    // default 0.1 seconds
    ros::Duration max_scan_age(ros::param::param<double>("max_scan_age", 0.1));

    // The last laser scan message that was received, or null if no message
    // has been received
    sensor_msgs::LaserScan::ConstPtr last_scan;

    // Receive laser scan messages
    auto laser_scan_callback = [&last_scan] (const sensor_msgs::LaserScan::ConstPtr& scan_message) {
        last_scan = scan_message;
    };
    auto subscription = handle.subscribe(laser_topic, 0, laser_callback_t(laser_scan_callback));


    // Store an occupancy grid map
    nav_msgs::OccupancyGrid occupancy_grid;
    // TODO: Populate

    // Respond to snapshot requests
    auto take_snapshot = [&last_scan, &occupancy_grid, max_scan_age] (mappi_msgs::Snapshot::Request&, mappi_msgs::Snapshot::Response& response) {
        ROS_INFO("Taking snapshot");
        if (!last_scan) {
            response.result = mappi_msgs::Snapshot::Response::RESULT_NO_LASER_DATA;
            return true;
        }
        const auto scan_age = ros::Time::now() - last_scan->header.stamp;
        if (scan_age >= max_scan_age) {
            response.result = mappi_msgs::Snapshot::Response::RESULT_STALE_LASER_DATA;
            return true;
        }

        response.result = mappi_msgs::Snapshot::Response::RESULT_OK;
        return true;
    };

    auto service = handle.advertiseService("snapshot", handler_callback_t(take_snapshot));

    ros::spin();

    return 0;
}
