#include "lane_map_msg_utils.h"
#include <tf/transform_datatypes.h>
#include "maps/utils/map_frame_utils.h"

namespace map_msg_utils {

perception_msgs::MapBoundary getBoundaryMsg(const lane_map::Boundary& boundary)
{
  perception_msgs::MapBoundary boundary_msg;
  boundary_msg.ref = getBoundaryRefMsg(boundary.ref);
  boundary_msg.style = boundary.style;
  boundary_msg.color = boundary.color;
  boundary_msg.material = boundary.material;
  boundary_msg.marking_width = boundary.marking_width;
  boundary_msg.is_only_emergency_boundary = boundary.is_only_emergency_boundary;
  boundary_msg.pts = boundary.pts;
  return boundary_msg;
}


perception_msgs::MapLane getLaneMsg(const lane_map::Lane& lane)
{
  perception_msgs::MapLane lane_msg;
  lane_msg.ref = getLaneRefMsg(lane.ref);
  lane_msg.lane_order = lane.lane_order;
  lane_msg.dot = basic_types::enum_value(lane.direction_of_travel);
  lane_msg.left_boundary_ref = getBoundaryRefMsg(lane.left_boundary_ref);
  lane_msg.right_boundary_ref = getBoundaryRefMsg(lane.right_boundary_ref);
  lane_msg.start_junction_ref = getJunctionRefMsg(lane.start_junction_ref);
  lane_msg.end_junction_ref = getJunctionRefMsg(lane.end_junction_ref);
  lane_msg.is_emergency_lane = lane.is_emergency_lane;
  lane_msg.merged = lane.merged;
  lane_msg.merging = lane.merging;
  lane_msg.upcoming_split = lane.upcoming_split;
  lane_msg.from_split = lane.from_split;
  lane_msg.pts = lane.pts;
  lane_msg.is_no_passing_zone = lane.is_no_passing_zone;
  lane_msg.lane_type = basic_types::enum_value(lane.lane_type);
  lane_msg.lane_transition_type = basic_types::enum_value(lane.lane_transition_type);
  return lane_msg;
}


perception_msgs::MapLaneGroup getLaneGroupMsg(const lane_map::LaneGroup& lane_group)
{
  perception_msgs::MapLaneGroup lg_msg;
  lg_msg.ref = getLaneGroupRefMsg(lane_group.ref);
  lg_msg.length = lane_group.length;
  lg_msg.dot = basic_types::enum_value(lane_group.direction_of_travel);
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

  lg_msg.functional_class = lane_group.functional_class;
  lg_msg.route_type = lane_group.route_type;
  lg_msg.is_accessible_by_pedestrians = lane_group.is_accessible_by_pedestrians;
  lg_msg.is_built_up_area_road = lane_group.is_built_up_area_road;
  lg_msg.is_controlled_access = lane_group.is_controlled_access;
  lg_msg.is_limited_access = lane_group.is_limited_access;
  lg_msg.is_multiply_digitized = lane_group.is_multiply_digitized;
  lg_msg.is_ramp = lane_group.is_ramp;
  lg_msg.is_tunnel = lane_group.is_tunnel;
  lg_msg.is_urban = lane_group.is_urban;
  lg_msg.is_within_interchange = lane_group.is_within_interchange;

  return lg_msg;
}

perception_msgs::MapLaneRef getLaneRefMsg(const lane_map::LaneRef& lane_ref)
{
  perception_msgs::MapLaneRef lane_ref_msg;
  lane_ref_msg.tile_id = lane_ref.tile_id;
  lane_ref_msg.lane_group_id = lane_ref.lg_id;
  lane_ref_msg.id = lane_ref.id;
  return lane_ref_msg;
}


perception_msgs::MapConnectorRef getConnectorRefMsg(const lane_map::ConnectorRef& conn_ref)
{
  perception_msgs::MapConnectorRef conn_ref_msg;
  conn_ref_msg.tile_id = conn_ref.tile_id;
  conn_ref_msg.id = conn_ref.id;
  return conn_ref_msg;
}

perception_msgs::MapJunctionRef getJunctionRefMsg(const lane_map::JunctionRef& junc_ref)
{
  perception_msgs::MapJunctionRef junc_ref_msg;
  junc_ref_msg.tile_id = junc_ref.tile_id;
  junc_ref_msg.connector_id = junc_ref.connector_id;
  junc_ref_msg.id = junc_ref.id;
  return junc_ref_msg;
}

perception_msgs::MapBoundaryRef getBoundaryRefMsg(const lane_map::BoundaryRef& boundary_ref)
{
  perception_msgs::MapBoundaryRef boundary_ref_msg;
  boundary_ref_msg.tile_id = boundary_ref.tile_id;
  boundary_ref_msg.lane_group_id = boundary_ref.lg_id;
  boundary_ref_msg.id = boundary_ref.id;
  return boundary_ref_msg;
}

perception_msgs::MapLaneGroupRef getLaneGroupRefMsg(const lane_map::LaneGroupRef& lg_ref)
{
  perception_msgs::MapLaneGroupRef lg_ref_msg;
  lg_ref_msg.tile_id = lg_ref.tile_id;
  lg_ref_msg.id = lg_ref.id;
  return lg_ref_msg;
}


perception_msgs::MapConnector getConnectorMsg(const lane_map::Connector& connector)
{
  perception_msgs::MapConnector connector_msg;
  connector_msg.ref = getConnectorRefMsg(connector.ref);
  connector_msg.boundary_geometry = connector.boundary_geometry;
  for (const auto& lg_ref : connector.connected_lane_groups) {
    connector_msg.connected_lane_groups.push_back(getLaneGroupRefMsg(lg_ref));
  }
  for (const auto& lg_ref : connector.inflow_refs) {
    connector_msg.inflow_refs.push_back(getLaneGroupRefMsg(lg_ref));
  }
  for (const auto& lg_ref : connector.outflow_refs) {
    connector_msg.outflow_refs.push_back(getLaneGroupRefMsg(lg_ref));
  }
  for (const auto& junction : connector.junctions) {
    connector_msg.junctions.push_back(getJunctionMsg(junction));
  }
  return connector_msg;
}


perception_msgs::MapJunction getJunctionMsg(const lane_map::Junction& junction)
{
  perception_msgs::MapJunction junction_msg;
  junction_msg.ref = getJunctionRefMsg(junction.ref);
  junction_msg.junction_type = junction.junction_type;
  junction_msg.pt = junction.pt;
  for (const auto& lg_ref : junction.inflow_refs) {
    junction_msg.inflow_refs.push_back(getLaneRefMsg(lg_ref));
  }
  for (const auto& lg_ref : junction.outflow_refs) {
    junction_msg.outflow_refs.push_back(getLaneRefMsg(lg_ref));
  }
  return junction_msg;
}


perception_msgs::MapTile getTileMsg(const lane_map::Tile& tile)
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

perception_msgs::Map getMapMsg(const maps::LaneSubMap& map)
{
  perception_msgs::Map map_msg;

  map_msg.map_frame = mapFrameMsgFromMapFrame(map.map_frame);

  // match header to map frame header
  map_msg.header = map_msg.map_frame.header;

  // add tiles
  for (const auto& tile_it : map.tiles) {
    const auto tile = tile_it.second;
    map_msg.tiles.push_back(getTileMsg(*tile));
  }
  return map_msg;
}

} // end namespace
