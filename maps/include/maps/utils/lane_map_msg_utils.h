#ifndef LANE_MAP_SERVER_MAPPERY_MAP_PARSER_H
#define LANE_MAP_SERVER_MAPPERY_MAP_PARSER_H

#include <geometry_msgs/Point.h>
#include <perception_msgs/Map.h>
#include <utils/map/utils.h>
#include <unordered_map>
#include "utils/map/structs.h"


namespace mappery {

class MapParser
{
 public:
  std::unordered_map<uint64_t, map_utils::Tile> getTiles(const perception_msgs::Map& map_msg);

 private:
  static map_utils::Tile getTileStruct(const perception_msgs::MapTile& tile_msg);

  static map_utils::LaneGroup
  getLaneGroupStruct(const perception_msgs::MapLaneGroup& lane_group_msg);

  static map_utils::Connector
  getConnectorStruct(const perception_msgs::MapConnector& connector_msg);

  static map_utils::ConnectorRef
  getConnectorRefStruct(const perception_msgs::MapConnectorRef& conn_ref_msg);

  static map_utils::Lane getLaneStruct(const perception_msgs::MapLane& lane_msg);

  static map_utils::Boundary getBoundaryStruct(const perception_msgs::MapBoundary& boundary_msg);

  static map_utils::LaneGroupRef
  getLaneGroupRefStruct(const perception_msgs::MapLaneGroupRef& lg_ref_msg);

  void addLaneGroupPerimeters(std::unordered_map<uint64_t, map_utils::Tile>& tiles);

  static void addGeoPts(std::vector<BoostPoint>& target, const std::vector<geometry_msgs::Point>& pts,
                        bool flip);
};

} // end namespace

#endif
