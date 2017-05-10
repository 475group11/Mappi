#include <cstdint>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mappi_msgs/Snapshot.h>

#include "mapper.h"

typedef boost::function<bool (mappi_msgs::Snapshot::Request&, mappi_msgs::Snapshot::Response&)> handler_callback_t;
typedef boost::function<void (const sensor_msgs::LaserScan::ConstPtr&)> laser_callback_t;

namespace {

/**
 * Gets an unsigned integer-type parameter
 *
 * Logs an error and exits if the parameter value is negative
 */
template <typename T>
bool get_unsigned_param(const std::string& name, int default_value) {
    const auto value = ros::param::param<int>(name, default_value);
    if (value < 0) {
        ROS_ERROR_STREAM("Negative value" << value << " for unsigned parameter " << name);
        std::exit(-2);
    }
    return static_cast<T>(value);
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mappi_lidar_snapshot_node");
    ros::NodeHandle handle;

    // Parameters

    // The topic to subscribe to for laser scan messages
    const auto laser_topic = ros::param::param<std::string>("~laser_topic", "/laser");
    // The global frame of reference
    const auto global_frame = ros::param::param<std::string>("~global_frame", "/map");
    // Width of the map, meters
    const auto map_width_m = get_unsigned_param<std::uint32_t>("~map_width_m", 10);
    // Height of the map, meters
    const auto map_height_m = get_unsigned_param<std::uint32_t>("~map_height_m", 10);
    // Map resolution, cells per meter
    const auto map_resolution = get_unsigned_param<std::uint32_t>("~map_resolution", 100);

    // The maximum age for a scan message before it is considered stale,
    // default 0.1 seconds
    const ros::Duration max_scan_age(ros::param::param<double>("~max_scan_age", 0.1));

    // The last laser scan message that was received, or null if no message
    // has been received
    sensor_msgs::LaserScan::ConstPtr last_scan;

    // Receive laser scan messages
    auto laser_scan_callback = [&last_scan] (const sensor_msgs::LaserScan::ConstPtr& scan_message) {
        last_scan = scan_message;
    };
    auto subscription = handle.subscribe(laser_topic, 0, laser_callback_t(laser_scan_callback));

    // Calculate occupancy grid parameters
    const auto grid_width_cells = map_width_m * map_resolution;
    const auto grid_height_cells = map_height_m * map_resolution;
    const auto grid_m_per_cell = 1.0f / static_cast<float>(map_resolution);

    // Store an occupancy grid map
    nav_msgs::OccupancyGrid occupancy_grid;
    // Set up grid
    occupancy_grid.info.resolution = grid_m_per_cell;
    occupancy_grid.info.width = grid_width_cells;
    occupancy_grid.info.height = grid_height_cells;
    // Origin is the global origin
    occupancy_grid.info.origin.position.x = 0;
    occupancy_grid.info.origin.position.y = 0;
    occupancy_grid.info.origin.position.z = 0;
    occupancy_grid.info.origin.orientation.x = 0;
    occupancy_grid.info.origin.orientation.y = 0;
    occupancy_grid.info.origin.orientation.z = 0;
    occupancy_grid.info.origin.orientation.w = 0;
    // Fill the grid with uncertainty (-1)
    occupancy_grid.data.clear();
    occupancy_grid.data.resize(grid_width_cells * grid_height_cells, -1);

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

        // TODO: Data

        response.result = mappi_msgs::Snapshot::Response::RESULT_OK;
        return true;
    };

    auto service = handle.advertiseService("snapshot", handler_callback_t(take_snapshot));

    ros::spin();

    return 0;
}
