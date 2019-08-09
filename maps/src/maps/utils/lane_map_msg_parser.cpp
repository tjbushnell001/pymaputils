#include "map_parser.h"

namespace mappery {

using map_utils::Tile;
using map_utils::LaneGroup;
using map_utils::Connector;
using map_utils::Lane;
using map_utils::Boundary;
using map_utils::LaneGroupRef;
using map_utils::ConnectorRef;


std::unordered_map<uint64_t, Tile> MapParser::getTiles(const perception_msgs::Map& map_msg)
{
  std::unordered_map<uint64_t, Tile> tiles;
  for (const auto& tile_msg : map_msg.tiles) {
    tiles[tile_msg.id] = getTileStruct(tile_msg);
  }
  addLaneGroupPerimeters(tiles);
  return tiles;
}


Tile MapParser::getTileStruct(const perception_msgs::MapTile& tile_msg)
{
  Tile tile;
  tile.id = tile_msg.id;
  for (const auto& lane_group : tile_msg.lane_groups) {
    tile.lane_groups[lane_group.id] = getLaneGroupStruct(lane_group);
  }
  for (const auto& connector : tile_msg.connectors) {
    tile.connectors[connector.id] = getConnectorStruct(connector);
  }
  return tile;
}


LaneGroup MapParser::getLaneGroupStruct(const perception_msgs::MapLaneGroup& lane_group_msg)
{
  LaneGroup lane_group;
  lane_group.id = lane_group_msg.id;
  lane_group.start_connector = getConnectorRefStruct(lane_group_msg.start_connector);
  lane_group.end_connector = getConnectorRefStruct(lane_group_msg.end_connector);
  lane_group.left_boundary = lane_group_msg.left_boundary;
  lane_group.right_boundary = lane_group_msg.right_boundary;
  for (const auto& lane_msg : lane_group_msg.lanes) {
    lane_group.lanes[lane_msg.id] = getLaneStruct(lane_msg);
  }
  for (const auto& boundary_msg : lane_group_msg.boundaries) {
    lane_group.boundaries[boundary_msg.id] = getBoundaryStruct(boundary_msg);
  }
  lane_group.is_right_shoulder_undrivable = lane_group_msg.is_right_shoulder_undrivable;
  return lane_group;
}


Connector MapParser::getConnectorStruct(const perception_msgs::MapConnector& connector_msg)
{
  Connector connector;
  connector.id = connector_msg.id;
  connector.boundary_geometry = connector_msg.boundary_geometry;
  for (const auto& lg_ref_msg : connector_msg.connected_lane_groups) {
    connector.connected_lane_groups.push_back(getLaneGroupRefStruct(lg_ref_msg));
  }
  return connector;
}


ConnectorRef MapParser::getConnectorRefStruct(const perception_msgs::MapConnectorRef& conn_ref_msg)
{
  ConnectorRef conn_ref;
  conn_ref.id = conn_ref_msg.id;
  conn_ref.tile_id = conn_ref_msg.tile_id;
  return conn_ref;
}


Lane MapParser::getLaneStruct(const perception_msgs::MapLane& lane_msg)
{
  Lane lane;
  lane.id = lane_msg.id;
  lane.pts = lane_msg.pts;
  lane.direction_of_travel = lane_msg.direction_of_travel;
  lane.left_boundary_id = lane_msg.left_boundary_id;
  lane.right_boundary_id = lane_msg.right_boundary_id;
  lane.start_connector_lane_number = lane_msg.start_connector_lane_number;
  lane.end_connector_lane_number = lane_msg.end_connector_lane_number;
  lane.is_emergency_lane = lane_msg.is_emergency_lane;
  lane.merged = lane_msg.merged;
  lane.merging = lane_msg.merging;
  lane.upcoming_split = lane_msg.upcoming_split;
  lane.from_split = lane_msg.from_split;
  lane.is_no_passing_zone = lane_msg.is_no_passing_zone;
  return lane;
}


Boundary MapParser::getBoundaryStruct(const perception_msgs::MapBoundary& boundary_msg)
{
  Boundary boundary;
  boundary.id = boundary_msg.id;
  boundary.pts = boundary_msg.pts;
  boundary.style = boundary_msg.style;
  boundary.color = boundary_msg.color;
  boundary.material = boundary_msg.material;
  boundary.marking_width = boundary_msg.marking_width;
  boundary.is_only_emergency_boundary = boundary_msg.is_only_emergency_boundary;
  return boundary;
}


LaneGroupRef MapParser::getLaneGroupRefStruct(const perception_msgs::MapLaneGroupRef& lg_ref_msg)
{
  LaneGroupRef lg_ref;
  lg_ref.tile_id = lg_ref_msg.tile_id;
  lg_ref.id = lg_ref_msg.id;
  return lg_ref;
}


void MapParser::addLaneGroupPerimeters(std::unordered_map<uint64_t, Tile>& tiles)
{
  for (auto& tile_pair : tiles) {
    for (auto& lg_pair : tile_pair.second.lane_groups) {
      std::vector<BoostPoint> pts;
      addGeoPts(pts, lg_pair.second.right_boundary, false);
      addGeoPts(pts, lg_pair.second.left_boundary, true);
      pts.push_back(pts[0]);
      BoostPolygon polygon;
      boost::geometry::assign_points(polygon, pts);
      lg_pair.second.perimeter = polygon;
    }
  }
  return;
}


void MapParser::addGeoPts(std::vector<BoostPoint>& target,
                          const std::vector<geometry_msgs::Point>& pts, bool flip)
{
  for (int i = 0; i < pts.size(); ++i) {
    int idx = (flip) ? pts.size() - i - 1 : i;
    BoostPoint tmp(pts[idx].x, pts[idx].y);
    target.push_back(tmp);
  }
  return;
}

} // end namespace
