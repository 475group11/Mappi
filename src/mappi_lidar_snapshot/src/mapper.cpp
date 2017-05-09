#include "mapper.h"

Mapper::Mapper() :
  _global_frame(ros::param::param<std::string>("~global_frame", "/map"))
{


    // Set up grid
    _map.info.resolution = grid_m_per_cell;
    _map.info.width = grid_width_cells;
    _map.info.height = grid_height_cells;
    // Origin is the global origin
    _map.info.origin.position.x = 0;
    _map.info.origin.position.y = 0;
    _map.info.origin.position.z = 0;
    _map.info.origin.orientation.x = 0;
    _map.info.origin.orientation.y = 0;
    _map.info.origin.orientation.z = 0;
    _map.info.origin.orientation.w = 0;
    // Fill the grid with uncertainty (-1)
    _map.data.clear();
    _map.data.resize(grid_width_cells * grid_height_cells, -1);
}
