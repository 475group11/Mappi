#include <string>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mappi_msgs/Snapshot.h>

/**
 * Uses transforms and laser scans to create a map
 */
class Mapper {
private:
    /**
     * The node handle
     */
    ros::NodeHandle _handle;

    /**
     * The name of the global frame in which to create the map
     */
    std::string _global_frame;

    /**
     * The current map
     */
    nav_msgs::OccupancyGrid _map;

public:
    Mapper();

};
