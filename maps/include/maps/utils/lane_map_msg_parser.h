#ifndef LANE_MAP_SERVER_MAPPERY_MAP_PARSER_H
#define LANE_MAP_SERVER_MAPPERY_MAP_PARSER_H

#include <geometry_msgs/Point.h>
#include <perception_msgs/Map.h>
#include <utils/map/utils.h>
#include <unordered_map>
#include "maps/map_layers.h"
#include "utils/map/lane_map_structs.h"

namespace map_msg_utils {

std::shared_ptr<maps::LaneSubMap> getMap(const perception_msgs::Map& map_msg);

std::unordered_map<uint64_t, std::shared_ptr<lane_map::Tile>>
getTiles(const perception_msgs::Map& map_msg);

lane_map::Tile getTileStruct(const perception_msgs::MapTile& tile_msg);

lane_map::LaneGroup getLaneGroupStruct(const perception_msgs::MapLaneGroup& lane_group_msg);

lane_map::Connector getConnectorStruct(const perception_msgs::MapConnector& connector_msg);

lane_map::ConnectorRef getConnectorRefStruct(const perception_msgs::MapConnectorRef& conn_ref_msg);

lane_map::Lane getLaneStruct(const perception_msgs::MapLane& lane_msg);

lane_map::Boundary getBoundaryStruct(const perception_msgs::MapBoundary& boundary_msg);

lane_map::LaneGroupRef getLaneGroupRefStruct(const perception_msgs::MapLaneGroupRef& lg_ref_msg);

lane_map::LaneRef getLaneRefStruct(const perception_msgs::MapLaneRef& lane_ref_msg);

lane_map::BoundaryRef getBoundaryRefStruct(const perception_msgs::MapBoundaryRef& boundary_ref_msg);

lane_map::Junction getJunctionStruct(const perception_msgs::MapJunction& junction_msg);

lane_map::JunctionRef getJunctionRefStruct(const perception_msgs::MapJunctionRef& junction_ref_msg);

} // namespace map_msg_utils

#endif
