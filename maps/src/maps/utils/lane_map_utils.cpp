#include "lane_map_utils.h"
#include <algorithm>
#include <sstream>
#include <stack>
#include <unordered_map>
#include "maps/utils/lane_map_filtering.h"
#include "utils/sets/set_utils.h"

using lane_map::Boundary;
using lane_map::BoundaryRef;
using lane_map::Connector;
using lane_map::ConnectorRef;
using lane_map::LaneJunction;
using lane_map::Lane;
using lane_map::LaneGroup;
using lane_map::LaneGroupRef;
using lane_map::LaneRef;

namespace lane_map_utils {
LaneJunction getForwardLaneJunction(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  const Lane* lane = map.getLane(lane_ref);
  assert(lane);

  if (!isDirectionalLane(*lane)) {
    throwDataError(lane_ref, "not directional");
  }

  LaneJunction lane_junction;
  lane_junction.ref = lane->end_junction_ref;

  lane_junction.outflowing_lanes = getConnectedLanes(map, lane_junction.ref, TraverseDirection::OUT);
  lane_junction.inflowing_lanes = getConnectedLanes(map, lane_junction.ref, TraverseDirection::IN);

  if (lane_junction.inflowing_lanes.empty()) {
    lane_junction.inflowing_lanes.push_back(lane_ref);
  }

  return lane_junction;
}


LaneJunction getBackwardLaneJunction(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  const Lane* lane = map.getLane(lane_ref);
  assert(lane);

  if (!isDirectionalLane(*lane)) {
    throwDataError(lane_ref, "not directional");
  }

  LaneJunction lane_junction;
  lane_junction.ref = lane->start_junction_ref;
  lane_junction.outflowing_lanes = getConnectedLanes(map, lane_junction.ref, TraverseDirection::OUT);
  lane_junction.inflowing_lanes = getConnectedLanes(map, lane_junction.ref, TraverseDirection::IN);

  if (lane_junction.outflowing_lanes.empty()) {
    lane_junction.outflowing_lanes.push_back(lane_ref);
  }

  return lane_junction;
}

std::unordered_set<LaneGroupRef> getConnectedLaneGroups(const maps::LaneSubMap& map,
                                                        const LaneGroup& lane_group,
                                                        TraverseDirection traverse_direction)
{
  switch (traverse_direction) {
    case TraverseDirection::BOTH: {
      auto forward = getConnectedLaneGroups(map, lane_group.end_connector, TraverseDirection::BOTH);
      auto backward = getConnectedLaneGroups(map, lane_group.start_connector, TraverseDirection::BOTH);

      return set_utils::set_union(forward, backward);
    } break;

    case TraverseDirection::IN:
      return getConnectedLaneGroups(map, lane_group.start_connector, TraverseDirection::IN);

    case TraverseDirection::OUT:
      return getConnectedLaneGroups(map, lane_group.end_connector, TraverseDirection::OUT);
    default:
      assert(false);
  }
}

std::unordered_set<LaneGroupRef> getReachableValidLaneGroups(const maps::LaneSubMap& map,
                                                             const LaneGroupRef& initial_lg_ref,
                                                             size_t max_distance)
{
  return lane_map_utils::traverseLaneGroups(map, initial_lg_ref, isDirectionalLaneGroup,
                                            TraverseDirection::BOTH, max_distance);
}


std::unordered_set<LaneGroupRef> getAllValidLaneGroups(const maps::LaneSubMap& map)
{
  /*
   * Returns all refs for all lane groups in the map which are valid
   */

  return lane_map_utils::filterLaneGroups(map, isDirectionalLaneGroup);
}

std::unordered_set<LaneGroupRef> getConnectedLaneGroups(const maps::LaneSubMap& map,
                                                        const ConnectorRef& conn_ref,
                                                        TraverseDirection traverse_direction)
{
  auto conn = map.getConnector(conn_ref);
  if (!conn) {
    return {};
  }

  switch (traverse_direction) {
    case TraverseDirection::BOTH:
      return conn->connected_lane_groups;

    case TraverseDirection::OUT:
      return conn->outflow_refs;

    case TraverseDirection::IN:
      return conn->inflow_refs;
  }
}


std::vector<LaneRef> getConnectedLanes(const maps::LaneSubMap& map,
                                       const lane_map::JunctionRef& junc_ref,
                                       TraverseDirection traverse_direction)
{
  const ConnectorRef conn_ref = junc_ref.getConnectorRef();
  const auto lg_refs = getConnectedLaneGroups(map, conn_ref);

  // Now we just figure out which of these are connected:
  std::vector<LaneRef> connected_lane_refs;
  for (const auto& lg_ref : lg_refs) {
    const LaneGroup* lg = map.getLaneGroup(lg_ref);
    if (!lg) {
      continue;
    }

    for (auto& lane_pair : lg->lanes) {
      const Lane& lane = lane_pair.second;

      bool match = false;
      switch (traverse_direction) {
        case TraverseDirection::OUT:
          match = lane.start_junction_ref == junc_ref;
          break;
        case TraverseDirection::IN:
          match = lane.end_junction_ref == junc_ref;
          break;
        case TraverseDirection::BOTH:
          match = (lane.start_junction_ref == junc_ref || lane.end_junction_ref == junc_ref);
          break;
      }
      if (match) {
        connected_lane_refs.push_back(lane_pair.first);
      }
    }
  }
  return connected_lane_refs;
}

std::vector<const Lane*> getLanesInOrder(const LaneGroup& lg)
{
  if (!isDirectionalLaneGroup(lg)) {
    return {};
  }

  std::vector<const Lane*> non_emergency_lanes;
  for (auto& lane_pair : lg.lanes) {
    const Lane* lane = &lane_pair.second;
    if (!lane->is_emergency_lane) {
      non_emergency_lanes.push_back(lane);
    }
  }

  // sort by lane_order
  std::sort(non_emergency_lanes.begin(), non_emergency_lanes.end(),
            [](const Lane* a, const Lane* b) -> bool { return a->lane_order < b->lane_order; });

  return non_emergency_lanes;
}


std::vector<LaneRef> getLanesToLeft(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  std::vector<LaneRef> lanes_to_the_left;
  const LaneGroupRef lg_ref = lane_ref.getLaneGroupRef();

  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg || !isDirectionalLaneGroup(*lg)) {
    return {};
  }

  const std::vector<const Lane*> left_to_right_lanes = getLanesInOrder(*lg);
  for (const auto i_lane : left_to_right_lanes) {
    if (i_lane->ref == lane_ref) {
      // stop when we hit lane_ref
      break;
    }
    lanes_to_the_left.push_back(i_lane->ref);
  }
  std::reverse(lanes_to_the_left.begin(), lanes_to_the_left.end());
  return lanes_to_the_left;
}


std::vector<LaneRef> getLanesToRight(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  std::vector<LaneRef> lanes_to_the_right;
  const LaneGroupRef lg_ref = lane_ref.getLaneGroupRef();

  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg || !isDirectionalLaneGroup(*lg)) {
    return {};
  }

  const std::vector<const Lane*> left_to_right_lanes = getLanesInOrder(*lg);
  bool seen_lane_ref = false;
  for (const auto i_lane : left_to_right_lanes) {
    if (i_lane->ref == lane_ref) {
      seen_lane_ref = true;
    } else if (seen_lane_ref) {
      // start after we hit lane ref
      lanes_to_the_right.push_back(i_lane->ref);
    }
  }
  return lanes_to_the_right;
}


BoundaryRef getRightBoundary(const Lane& lane)
{
  if (!isDirectionalLane(lane)) {
    throwDataError(lane.ref, "getRightBoundary - lane has not a valid direction!");
  }
  return lane.right_boundary_ref;
}


BoundaryRef getLeftBoundary(const Lane& lane)
{
  if (!isDirectionalLane(lane)) {
    throwDataError(lane.ref, "getLeftBoundary - lane has not a valid direction!");
  }
  return lane.left_boundary_ref;
}


std::vector<geometry_msgs::Point> getLanePtsForward(const maps::LaneSubMap& map,
                                                    const LaneRef& lane_ref)
{
  const Lane* lane = map.getLane(lane_ref);
  if (!lane) {
    return {};
  }
  return getLanePtsForward(*lane);
}


std::vector<geometry_msgs::Point> getLanePtsForward(const Lane& lane)
{
  if (!isDirectionalLane(lane)) {
    return {};
  }

  return lane.pts;
}


std::vector<geometry_msgs::Point> getBoundaryPtsForward(const maps::LaneSubMap& map,
                                                        const Boundary& boundary)
{
  const LaneGroupRef lg_ref = boundary.ref.getLaneGroupRef();
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  assert(lg);

  if (!isDirectionalLaneGroup(*lg)) {
    throwDataError(lg_ref, "not directional");
  }

  return boundary.pts;
}

bool isDirectionalLane(const Lane& lane)
{
  return lane.direction_of_travel == lane_map::DirectionOfTravel::FORWARD;
}

bool isDirectionalLaneGroup(const LaneGroup& lg)
{
  return lg.direction_of_travel == lane_map::DirectionOfTravel::FORWARD;
}

std::unordered_set<LaneGroupRef> getLaneGroupsAtLocation(const maps::LaneSubMap& map, double x_loc,
                                                         double y_loc)
{
  const BoostPoint pt(x_loc, y_loc);
  auto filter_fn = [&pt](const lane_map::LaneGroup& lg) {
    return boost::geometry::within(pt, lg.perimeter);
  };
  return lane_map_utils::filterLaneGroups(map, filter_fn);
}

std::unordered_set<LaneRef> getAllValidLanes(const maps::LaneSubMap& map)
{
  auto filter_fn = [](const Lane& lane) {
    return (!lane.is_emergency_lane && isDirectionalLane(lane));
  };
  return lane_map_utils::filterLanes(map, filter_fn);
}


bool isOvertakingAllowed(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  const Lane* lane = map.getLane(lane_ref);
  if (!lane) {
    throwDataError(lane_ref, "Given lane does not exist");
  }

  return !lane->is_no_passing_zone;
}


bool isDrivingOnRightShoulderAllowed(const maps::LaneSubMap& map, const LaneRef& lane_ref)
{
  const LaneGroupRef& lg_ref = lane_ref.getLaneGroupRef();
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }

  return !lg->is_right_shoulder_undrivable;
}


BoostPolygon getNonEmergencyLaneGroupPerimeter(const maps::LaneSubMap& map,
                                               const LaneGroupRef& lg_ref)
{
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }

  // Extract left boundary of left most lane, and right boundary of right most lane:
  std::vector<const Lane*> in_order = getLanesInOrder(*lg);
  assert(!in_order.empty());
  const Lane* l_lane = in_order.front();
  const Lane* r_lane = in_order.back();

  if (!l_lane) {
    throwDataError(l_lane->ref, "Given lane does not exist");
  }
  if (!r_lane) {
    throwDataError(r_lane->ref, "Given lane does not exist");
  }

  const BoundaryRef l_boundary_ref = getLeftBoundary(*l_lane);
  const BoundaryRef r_boundary_ref = getRightBoundary(*r_lane);

  const Boundary* l_boundary = map.getBoundary(l_boundary_ref);
  const Boundary* r_boundary = map.getBoundary(r_boundary_ref);
  assert(l_boundary);
  assert(r_boundary);

  const std::vector<geometry_msgs::Point> l_side = getBoundaryPtsForward(map, *l_boundary);
  const std::vector<geometry_msgs::Point> r_side = getBoundaryPtsForward(map, *r_boundary);

  std::vector<BoostPoint> perimeter_pts;
  // Append left boundary forwards:
  for (auto it = r_side.begin(); it != r_side.end(); ++it) {
    BoostPoint pt(it->x, it->y);
    perimeter_pts.push_back(pt);
  }
  // Append right boundary backwards:
  for (auto it = l_side.rbegin(); it != l_side.rend(); ++it) {
    BoostPoint pt(it->x, it->y);
    perimeter_pts.push_back(pt);
  }
  // Append last point to the end to close the loop:
  perimeter_pts.push_back(perimeter_pts[0]);

  // Output:
  BoostPolygon perimeter_polygon;
  boost::geometry::assign_points(perimeter_polygon, perimeter_pts);
  return perimeter_polygon;
}

std::vector<geometry_msgs::Point> getNonEmergencyLeftLaneGroupBoundary(const maps::LaneSubMap& map,
                                                                       const LaneGroupRef& lg_ref)
{
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }

  // Extract left boundary of left most lane:
  const std::vector<const Lane*> in_order = getLanesInOrder(*lg);
  assert(!in_order.empty());
  const Lane* l_lane = in_order.front();
  if (!l_lane) {
    throwDataError(l_lane->ref, "Given lane does not exist");
  }

  const BoundaryRef l_boundary_ref = getLeftBoundary(*l_lane);
  const Boundary* l_boundary = map.getBoundary(l_boundary_ref);
  assert(l_boundary);

  return getBoundaryPtsForward(map, *l_boundary);
}

std::vector<geometry_msgs::Point> getNonEmergencyRightLaneGroupBoundary(const maps::LaneSubMap& map,
                                                                        const LaneGroupRef& lg_ref)
{
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }

  // Extract right boundary of right most lane:
  const std::vector<const Lane*> in_order = getLanesInOrder(*lg);
  assert(!in_order.empty());

  const Lane* r_lane = in_order.back();
  if (!r_lane) {
    throwDataError(r_lane->ref, "Given lane does not exist");
  }

  const BoundaryRef r_boundary_ref = getRightBoundary(*r_lane);
  const Boundary* r_boundary = map.getBoundary(r_boundary_ref);
  assert(r_boundary);

  return getBoundaryPtsForward(map, *r_boundary);
}

std::unordered_set<ConnectorRef> getAllConnectedConnectors(const maps::LaneSubMap& map,
                                                           const LaneGroupRef& lg_ref)
{
  std::unordered_set<ConnectorRef> output;
  const LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    return {};
  }

  if (map.getConnector(lg->start_connector)) {
    output.insert(lg->start_connector);
  }
  if (map.getConnector(lg->end_connector)) {
    output.insert(lg->end_connector);
  }
  return output;
}

std::unordered_set<LaneGroupRef> getLaneGroupsInZone(const maps::LaneSubMap& map,
                                                     const BoostPolygon& zone)
{
  auto filter_fn = [&zone](const lane_map::LaneGroup& lg) {
    return boost::geometry::intersects(zone, lg.perimeter);
  };
  return lane_map_utils::filterLaneGroups(map, filter_fn);
}

std::vector<BoundaryRef> getBoundariesInZone(const maps::LaneSubMap& map,
                                             const std::vector<BoostPoint>& zone_pts)
{
  // Generate boost geometry:
  BoostPolygon poly_zone;
  BoostLinestring line_zone;
  boost::geometry::assign_points(poly_zone, zone_pts);
  boost::geometry::assign_points(line_zone, zone_pts);

  std::vector<BoundaryRef> boundaries_in_zone;
  const auto lg_refs = getLaneGroupsInZone(map, poly_zone);
  for (const auto& lg_ref : lg_refs) {
    const LaneGroup* lg = map.getLaneGroup(lg_ref);
    assert(lg);
    for (const auto& boundary_pair : lg->boundaries) {
      const Boundary& boundary = boundary_pair.second;
      // Check to see if any of the boundary is in or passes through the zone:
      bool in_zone = false;
      bool intersects = false;
      if (boundary.is_only_emergency_boundary) {
        continue;
      }
      std::vector<BoostPoint> boundary_as_boost_pts;
      for (const auto& pt : boundary.pts) {
        const BoostPoint pt_check(pt.x, pt.y);
        const bool is_inside = boost::geometry::within(pt_check, poly_zone);
        if (is_inside) {
          in_zone = true;
          break;
        }
        boundary_as_boost_pts.push_back(pt_check);
      }
      if (!in_zone) {
        BoostLinestring line_check;
        boost::geometry::assign_points(line_check, boundary_as_boost_pts);
        intersects = boost::geometry::intersects(line_check, line_zone);
      }
      if (in_zone || intersects) {
        boundaries_in_zone.push_back(boundary_pair.first);
      }
    }
  }
  return boundaries_in_zone;
}

LaneRef createLaneRefFromLaneGroupRef(const LaneGroupRef& lg_ref, uint8_t lane_id)
{
  return LaneRef(lg_ref.tile_id, lg_ref.id, lane_id);
}

size_t getNumberOfLanesInLaneGroup(const LaneGroup& lg)
{
  size_t n_lanes = 0;
  for (const auto& lane_pair : lg.lanes) {
    if (!lane_pair.second.is_emergency_lane) {
      ++n_lanes;
    }
  }
  return n_lanes;
}

std::shared_ptr<maps::LaneSubMap> copySubMap(
    const maps::LaneSubMap& map, const std::unordered_set<lane_map::LaneGroupRef>& lane_groups)
{
  auto new_map = std::make_shared<maps::LaneSubMap>(map.map_frame.type);
  new_map->map_frame = map.map_frame;

  // Add the specified lane groups
  for (const auto& lg_ref : lane_groups) {
    auto lg = map.getLaneGroup(lg_ref);
    if (lg) {
      // lane group exist
      new_map->addLaneGroup(lg_ref, *lg);

      // Add any connectors which contain links to these lane groups
      for (const auto& conn_ref : { lg->start_connector, lg->end_connector }) {
        if (new_map->getConnector(conn_ref)) {
          // already has connector
          continue;
        }

        auto conn = map.getConnector(conn_ref);
        if (conn) {
          // connector exists
          new_map->addConnector(conn_ref, *conn);
        }
      }
    }
  }
  return new_map;
}

std::unordered_set<lane_map::LaneGroupRef> getNearbyValidLaneGroupsGCS(const maps::LaneSubMap& map,
                                                                       double lat, double lng,
                                                                       double max_distance)
{
  assert(map.map_frame.type == maps::MapFrameType::GCS ||
         map.map_frame.type == maps::MapFrameType::GCS_NED);
  const bool use_NED = map.map_frame.type == maps::MapFrameType::GCS_NED;

  auto filter_fn = [lat, lng, use_NED, max_distance](const lane_map::LaneGroup& lg) {
    return (map_utils::haversineDistance(lg, lat, lng, use_NED) <= max_distance &&
            isDirectionalLaneGroup(lg));
  };

  return lane_map_utils::filterLaneGroups(map, filter_fn);
}

std::unordered_set<lane_map::LaneGroupRef> getNearbyValidLaneGroups(const maps::LaneSubMap& map,
                                                                    const geometry_msgs::Point& pt,
                                                                    double max_distance)
{
  assert(map.map_frame.type == maps::MapFrameType::UTM ||
         map.map_frame.type == maps::MapFrameType::VEHICLE);

  auto filter_fn = [pt, max_distance](const lane_map::LaneGroup& lg) {
    const double d = std::min(geometric::minDistanceToPiecewiseLine(pt.x, pt.y, lg.left_boundary),
                              geometric::minDistanceToPiecewiseLine(pt.x, pt.y, lg.right_boundary));
    return (d <= max_distance && isDirectionalLaneGroup(lg));
  };
  return lane_map_utils::filterLaneGroups(map, filter_fn);
}

void throwDataError(const LaneRef& lane_ref, const std::string& error_msg)
{
  std::stringstream ss;
  ss << error_msg;
  ss << ", lane_ref: " << lane_ref;
  throw MapDataError(ss.str());
}


void throwDataError(const LaneGroupRef& lg_ref, const std::string& error_msg)
{
  std::stringstream ss;
  ss << error_msg;
  ss << ", lane_group_ref: " << lg_ref;
  throw MapDataError(ss.str());
}

} // namespace lane_map_utils
