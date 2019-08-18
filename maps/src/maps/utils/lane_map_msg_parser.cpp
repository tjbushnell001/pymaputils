#include "lane_map_msg_parser.h"
#include "maps/utils/map_frame_utils.h"
#include "utils/map/lane_map_parser.h"

namespace map_msg_utils {

using lane_map::Tile;
using lane_map::LaneGroup;
using lane_map::Connector;
using lane_map::Junction;
using lane_map::Lane;
using lane_map::Boundary;
using lane_map::LaneGroupRef;
using lane_map::ConnectorRef;
using lane_map::JunctionRef;
using lane_map::BoundaryRef;
using lane_map::LaneRef;

std::shared_ptr<maps::LaneSubMap> getMap(const perception_msgs::Map& map_msg)
{
  const maps::MapFrame map_frame = maps::mapFrameFromMapFrameMsg(map_msg.map_frame);

  // set map frame
  auto map = std::make_shared<maps::LaneSubMap>(map_frame.type);
  map->map_frame = map_frame;

  // parse tiles
  map->tiles = getTiles(map_msg);

  return map;
}

std::unordered_map<uint64_t, std::shared_ptr<lane_map::Tile>>
getTiles(const perception_msgs::Map& map_msg)
{
  std::unordered_map<uint64_t, std::shared_ptr<lane_map::Tile>> tiles;
  for (const auto& tile_msg : map_msg.tiles) {
    auto tile = std::make_shared<lane_map::Tile>(getTileStruct(tile_msg));
    tiles[tile_msg.id] = tile;
  }
  return tiles;
}


Tile getTileStruct(const perception_msgs::MapTile& tile_msg)
{
  Tile tile;
  tile.id = tile_msg.id;
  for (const auto& lane_group_msg : tile_msg.lane_groups) {
    lane_map::LaneGroup lane_group = getLaneGroupStruct(lane_group_msg);

    // if possible, normalize direction of travel to FORWARD
    normalizeDirectionOfTravel(&lane_group);

    tile.lane_groups[lane_group.ref] = lane_group;
  }
  for (const auto& connector_msg : tile_msg.connectors) {
    const lane_map::Connector connector = getConnectorStruct(connector_msg);
    tile.connectors[connector.ref] = connector;
  }

  return tile;
}


LaneGroup getLaneGroupStruct(const perception_msgs::MapLaneGroup& lane_group_msg)
{
  LaneGroup lane_group;
  lane_group.ref = getLaneGroupRefStruct(lane_group_msg.ref);
  lane_group.start_connector = getConnectorRefStruct(lane_group_msg.start_connector);
  lane_group.end_connector = getConnectorRefStruct(lane_group_msg.end_connector);
  lane_group.direction_of_travel =
      static_cast<lane_map::DirectionOfTravel>(lane_group_msg.direction_of_travel);

  lane_group.left_boundary = lane_group_msg.left_boundary;
  lane_group.right_boundary = lane_group_msg.right_boundary;
  for (const auto& lane_msg : lane_group_msg.lanes) {
    lane_map::Lane lane = getLaneStruct(lane_msg);
    lane_group.lanes[lane.ref] = lane;
  }
  for (const auto& boundary_msg : lane_group_msg.boundaries) {
    lane_map::Boundary boundary = getBoundaryStruct(boundary_msg);
    lane_group.boundaries[boundary.ref] = boundary;
  }
  lane_group.is_right_shoulder_undrivable = lane_group_msg.is_right_shoulder_undrivable;

  lane_group.functional_class = lane_group_msg.functional_class;
  lane_group.route_type = lane_group_msg.route_type;
  lane_group.is_accessible_by_pedestrians = lane_group_msg.is_accessible_by_pedestrians;
  lane_group.is_built_up_area_road = lane_group_msg.is_built_up_area_road;
  lane_group.is_controlled_access = lane_group_msg.is_controlled_access;
  lane_group.is_limited_access = lane_group_msg.is_limited_access;
  lane_group.is_multiply_digitized = lane_group_msg.is_multiply_digitized;
  lane_group.is_ramp = lane_group_msg.is_ramp;
  lane_group.is_tunnel = lane_group_msg.is_tunnel;
  lane_group.is_urban = lane_group_msg.is_urban;
  lane_group.is_within_interchange = lane_group_msg.is_within_interchange;

  lane_group.perimeter =
      lane_map::createPerimeterPolygon(lane_group.left_boundary, lane_group.right_boundary);
  return lane_group;
}


Connector getConnectorStruct(const perception_msgs::MapConnector& connector_msg)
{
  Connector connector;
  connector.ref = getConnectorRefStruct(connector_msg.ref);
  connector.boundary_geometry = connector_msg.boundary_geometry;
  for (const auto& lg_ref_msg : connector_msg.connected_lane_groups) {
    connector.connected_lane_groups.insert(getLaneGroupRefStruct(lg_ref_msg));
  }
  for (const auto& lg_ref_msg : connector_msg.inflow_refs) {
    connector.inflow_refs.insert(getLaneGroupRefStruct(lg_ref_msg));
  }
  for (const auto& lg_ref_msg : connector_msg.outflow_refs) {
    connector.outflow_refs.insert(getLaneGroupRefStruct(lg_ref_msg));
  }

  for (const auto& junction_msg : connector_msg.junctions) {
    connector.junctions.push_back(getJunctionStruct(junction_msg));
  }

  return connector;
}


ConnectorRef getConnectorRefStruct(const perception_msgs::MapConnectorRef& conn_ref_msg)
{
  return ConnectorRef(conn_ref_msg.tile_id, conn_ref_msg.id);
}


Lane getLaneStruct(const perception_msgs::MapLane& lane_msg)
{
  Lane lane;
  lane.ref = getLaneRefStruct(lane_msg.ref);
  lane.pts = lane_msg.pts;
  lane.lane_order = lane_msg.lane_order;
  lane.direction_of_travel = static_cast<lane_map::DirectionOfTravel>(lane_msg.direction_of_travel);
  lane.left_boundary_ref = getBoundaryRefStruct(lane_msg.left_boundary_ref);
  lane.right_boundary_ref = getBoundaryRefStruct(lane_msg.right_boundary_ref);
  lane.start_junction_ref = getJunctionRefStruct(lane_msg.start_junction_ref);
  lane.end_junction_ref = getJunctionRefStruct(lane_msg.end_junction_ref);
  lane.is_emergency_lane = lane_msg.is_emergency_lane;
  lane.merged = lane_msg.merged;
  lane.merging = lane_msg.merging;
  lane.upcoming_split = lane_msg.upcoming_split;
  lane.from_split = lane_msg.from_split;
  lane.is_no_passing_zone = lane_msg.is_no_passing_zone;
  lane.lane_type = static_cast<lane_map::LaneType>(lane_msg.lane_type);
  lane.lane_transition_type =
      static_cast<lane_map::LaneTransitionType>(lane_msg.lane_transition_type);
  return lane;
}

LaneRef getLaneRefStruct(const perception_msgs::MapLaneRef& lane_ref_msg)
{
  return LaneRef(lane_ref_msg.tile_id, lane_ref_msg.lane_group_id, lane_ref_msg.id);
}

Boundary getBoundaryStruct(const perception_msgs::MapBoundary& boundary_msg)
{
  Boundary boundary;
  boundary.ref = getBoundaryRefStruct(boundary_msg.ref);
  boundary.pts = boundary_msg.pts;
  boundary.style = boundary_msg.style;
  boundary.color = boundary_msg.color;
  boundary.material = boundary_msg.material;
  boundary.marking_width = boundary_msg.marking_width;
  boundary.is_only_emergency_boundary = boundary_msg.is_only_emergency_boundary;
  return boundary;
}

BoundaryRef getBoundaryRefStruct(const perception_msgs::MapBoundaryRef& boundary_ref_msg)
{
  return BoundaryRef(boundary_ref_msg.tile_id, boundary_ref_msg.lane_group_id, boundary_ref_msg.id);
}


LaneGroupRef getLaneGroupRefStruct(const perception_msgs::MapLaneGroupRef& lg_ref_msg)
{
  return LaneGroupRef(lg_ref_msg.tile_id, lg_ref_msg.id);
}


Junction getJunctionStruct(const perception_msgs::MapJunction& junction_msg)
{
  lane_map::Junction junction;
  junction.ref = getJunctionRefStruct(junction_msg.ref);
  junction.pt = junction_msg.pt;
  junction.junction_type = junction_msg.junction_type;

  for (const auto& lane_ref_msg : junction_msg.inflow_refs) {
    junction.inflow_refs.insert(getLaneRefStruct(lane_ref_msg));
  }
  for (const auto& lane_ref_msg : junction_msg.outflow_refs) {
    junction.outflow_refs.insert(getLaneRefStruct(lane_ref_msg));
  }
  return junction;
}

JunctionRef getJunctionRefStruct(const perception_msgs::MapJunctionRef& junction_ref_msg)
{
  return JunctionRef(junction_ref_msg.tile_id, junction_ref_msg.connector_id, junction_ref_msg.id);
}

} // end namespace
