#include "map_msg_utils.h"


namespace map_msg_utils {


namespace ms = map_utils;


perception_msgs::MapBoundary getBoundaryMsg(const ms::Boundary& boundary)
{
  perception_msgs::MapBoundary boundary_msg;
  boundary_msg.id = boundary.id;
  boundary_msg.style = boundary.style;
  boundary_msg.color = boundary.color;
  boundary_msg.material = boundary.material;
  boundary_msg.marking_width = boundary.marking_width;
  boundary_msg.is_only_emergency_boundary = boundary.is_only_emergency_boundary;
  boundary_msg.pts = boundary.pts;
  return boundary_msg;
}


perception_msgs::MapLane getLaneMsg(const ms::Lane& lane)
{
  perception_msgs::MapLane lane_msg;
  lane_msg.id = lane.id;
  lane_msg.direction_of_travel = lane.direction_of_travel;
  lane_msg.left_boundary_id = lane.left_boundary_id;
  lane_msg.right_boundary_id = lane.right_boundary_id;
  lane_msg.start_connector_lane_number = lane.start_connector_lane_number;
  lane_msg.end_connector_lane_number = lane.end_connector_lane_number;
  lane_msg.is_emergency_lane = lane.is_emergency_lane;
  lane_msg.merged = lane.merged;
  lane_msg.merging = lane.merging;
  lane_msg.upcoming_split = lane.upcoming_split;
  lane_msg.from_split = lane.from_split;
  lane_msg.pts = lane.pts;
  lane_msg.is_no_passing_zone = lane.is_no_passing_zone;
  return lane_msg;
}


perception_msgs::MapLaneGroup getLaneGroupMsg(const ms::LaneGroup& lane_group)
{
  perception_msgs::MapLaneGroup lg_msg;
  lg_msg.id = lane_group.id;
  lg_msg.length = lane_group.length;
  lg_msg.left_boundary = lane_group.left_boundary;
  lg_msg.right_boundary = lane_group.right_boundary;
  lg_msg.start_connector = getConnectorRefMsg(lane_group.start_connector);
  lg_msg.end_connector = getConnectorRefMsg(lane_group.end_connector);
  for (const auto& lane_it : lane_group.lanes) {
    lg_msg.lanes.push_back(getLaneMsg(lane_it.second));
  }
  for (const auto& boundary_it : lane_group.boundaries) {
    lg_msg.boundaries.push_back(getBoundaryMsg(boundary_it.second));
  }
  lg_msg.is_right_shoulder_undrivable = lane_group.is_right_shoulder_undrivable;
  return lg_msg;
}


perception_msgs::MapConnectorRef getConnectorRefMsg(const ms::ConnectorRef& conn_ref)
{
  perception_msgs::MapConnectorRef conn_ref_msg;
  conn_ref_msg.tile_id = conn_ref.tile_id;
  conn_ref_msg.id = conn_ref.id;
  return conn_ref_msg;
}


perception_msgs::MapLaneGroupRef getLaneGroupRefMsg(const ms::LaneGroupRef& lg_ref)
{
  perception_msgs::MapLaneGroupRef lg_ref_msg;
  lg_ref_msg.tile_id = lg_ref.tile_id;
  lg_ref_msg.id = lg_ref.id;
  return lg_ref_msg;
}


perception_msgs::MapConnector getConnectorMsg(const ms::Connector& connector)
{
  perception_msgs::MapConnector connector_msg;
  connector_msg.id = connector.id;
  connector_msg.boundary_geometry = connector.boundary_geometry;
  for (const auto& lg_ref : connector.connected_lane_groups) {
    connector_msg.connected_lane_groups.push_back(getLaneGroupRefMsg(lg_ref));
  }
  for (const auto& junction : connector.junctions) {
    connector_msg.junctions.push_back(getJunctionMsg(junction));
  }
  return connector_msg;
}


perception_msgs::MapJunction getJunctionMsg(const ms::Junction& junction)
{
  perception_msgs::MapJunction junction_msg;
  junction_msg.id = junction.id;
  junction_msg.junction_type = junction.junction_type;
  junction_msg.pt = junction.pt;
  return junction_msg;
}


perception_msgs::MapTile getTileMsg(const ms::Tile& tile)
{
  perception_msgs::MapTile tile_msg;
  tile_msg.id = tile.id;
  for (const auto& lg_it : tile.lane_groups) {
    tile_msg.lane_groups.push_back(getLaneGroupMsg(lg_it.second));
  }
  for (const auto& conn_it : tile.connectors) {
    tile_msg.connectors.push_back(getConnectorMsg(conn_it.second));
  }
  return tile_msg;
}


perception_msgs::Map getMapMsg(const ms::Map& map_struct)
{
  perception_msgs::Map map_msg;
  for (const auto& tile_it : map_struct.tiles) {
    map_msg.tiles.push_back(getTileMsg(tile_it.second));
  }
  return map_msg;
}


} // end namespace
