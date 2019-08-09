#include "map_transverser.h"
#include <algorithm>
#include <sstream>
#include <stack>
#include <unordered_map>
#include "boost/assign/std/vector.hpp"

namespace mappery {

using map_utils::Boundary;
using map_utils::BoundaryRef;
using map_utils::Connector;
using map_utils::ConnectorRef;
using map_utils::LaneJunction;
using map_utils::Lane;
using map_utils::LaneGroup;
using map_utils::LaneGroupRef;
using map_utils::LaneRef;

MapTransverser::MapTransverser(const perception_msgs::Map& map_msg) : map_(map_msg)
{
  // doing everything in initialization list.
}


MapTransverser::MapTransverser(const map_utils::Map& map) : map_(map)
{
  // doing everything in initialization list.
}


LaneJunction MapTransverser::getForwardLaneJunction(const LaneRef& lane_ref) const
{
  LaneGroupRef lg_ref = map_.getLaneGroupRef(lane_ref);
  if (!isDirectionalLaneGroup(lg_ref)) {
    throwDataError(lg_ref, "not directional");
  }

  LaneJunction lane_junction;
  ConnectorRef next_connector_ref = getForwardConnectorRef(lg_ref);
  if (map_.connectorExists(next_connector_ref)) {
    const Lane& current_lane = map_.getLaneFromRef(lane_ref);
    int connected_number = getForwardLaneConnectorLaneNumber(lane_ref);

    std::string out_dir = "OUTFLOWING";
    std::vector<LaneRef> outflowing_lanes;
    outflowing_lanes = getConnectedLanes(next_connector_ref, connected_number, out_dir);
    lane_junction.outflowing_lanes = outflowing_lanes;

    std::string in_dir = "INFLOWING";
    std::vector<LaneRef> inflowing_lanes;
    inflowing_lanes = getConnectedLanes(next_connector_ref, connected_number, in_dir);
    lane_junction.inflowing_lanes = inflowing_lanes;
  } else {
    lane_junction.inflowing_lanes.push_back(lane_ref);
  }

  return lane_junction;
}


LaneJunction MapTransverser::getBackwardLaneJunction(const LaneRef& lane_ref) const
{
  LaneGroupRef lg_ref = map_.getLaneGroupRef(lane_ref);
  if (!isDirectionalLaneGroup(lg_ref)) {
    throwDataError(lg_ref, "not directional");
  }

  LaneJunction lane_junction;
  ConnectorRef next_connector_ref = getBackwardConnectorRef(lg_ref);
  if (map_.connectorExists(next_connector_ref)) {
    const Lane& current_lane = map_.getLaneFromRef(lane_ref);
    int connected_number = getBackwardLaneConnectorLaneNumber(lane_ref);

    std::string out_dir = "OUTFLOWING";
    std::vector<LaneRef> outflowing_lanes;
    outflowing_lanes = getConnectedLanes(next_connector_ref, connected_number, out_dir);
    lane_junction.outflowing_lanes = outflowing_lanes;

    std::string in_dir = "INFLOWING";
    std::vector<LaneRef> inflowing_lanes;
    inflowing_lanes = getConnectedLanes(next_connector_ref, connected_number, in_dir);
    lane_junction.inflowing_lanes = inflowing_lanes;
  } else {
    lane_junction.outflowing_lanes.push_back(lane_ref);
  }

  return lane_junction;
}


std::vector<LaneGroupRef> MapTransverser::getConnectedLaneGroups(const ConnectorRef& conn_ref) const
{
  if (map_.connectorExists(conn_ref)) {
    const Connector& conn = map_.getConnectorFromRef(conn_ref);
    return conn.connected_lane_groups;
  } else {
    std::vector<LaneGroupRef> connected_lg_refs;
    return connected_lg_refs;
  }
}


std::vector<LaneGroupRef> MapTransverser::getConnectedUnidirectionalLaneGroupsValidInMap(
    const LaneGroupRef& input_lg_ref) const
{
  /*
   *  Get all lane groups which are connected to the current lane group via either
   *  of its connectors but only if they are in the current map.
   */

  // First add connectors which are connected to input lane group and exist in the map:
  std::vector<ConnectorRef> connector_refs;
  if (map_.laneGroupExists(input_lg_ref)) {
    // Add front connector if it exists:
    ConnectorRef forward_conn_ref = getForwardConnectorRef(input_lg_ref);
    if (map_.connectorExists(forward_conn_ref)) {
      connector_refs.push_back(forward_conn_ref);
    }
    // Add backward connector if it exists:
    ConnectorRef backward_conn_ref = getBackwardConnectorRef(input_lg_ref);
    if (map_.connectorExists(backward_conn_ref)) {
      connector_refs.push_back(backward_conn_ref);
    }
  }

  // Now  all connector in "connector_refs" should be valid. Push back all lane groups
  // which are valid and are connected to these valid connectors:
  std::vector<LaneGroupRef> connected_lane_groups;
  for (const ConnectorRef& conn_ref : connector_refs) {
    const Connector& connector = map_.getConnectorFromRef(conn_ref);
    for (const LaneGroupRef& i_lg_ref : connector.connected_lane_groups) {
      if (map_.laneGroupExists(i_lg_ref) && isDirectionalLaneGroup(i_lg_ref)) {
        if (!(i_lg_ref == input_lg_ref)) {
          connected_lane_groups.push_back(i_lg_ref);
        }
      }
    }
  }
  return connected_lane_groups;
}


std::vector<LaneGroupRef>
MapTransverser::getReachableLaneGroupsValidInMap(const LaneGroupRef& input_lg_ref) const
{
  /*
   * Using an input lane group, it returns all lane groups which can be reached by visiting
   * the connector-lane group graph from this connected (in any direction).
   * It only returns lane groups which exist in the current map_
   */
  std::vector<LaneGroupRef> reachable_lane_groups;
  std::unordered_map<LaneGroupRef, bool> visited_lane_groups;
  std::stack<LaneGroupRef> lane_groups_to_visit;

  if (map_.laneGroupExists(input_lg_ref)) {
    lane_groups_to_visit.push(input_lg_ref);
  }

  // recursively add lane groups which are connected
  while (lane_groups_to_visit.size() > 0) {
    LaneGroupRef lg_ref = lane_groups_to_visit.top();
    lane_groups_to_visit.pop();
    visited_lane_groups[lg_ref] = true;
    reachable_lane_groups.push_back(lg_ref);
    // Only proceed if the lane group exists and it has not been visited.
    std::vector<LaneGroupRef> connected_lgs;
    connected_lgs = getConnectedUnidirectionalLaneGroupsValidInMap(lg_ref);
    for (const LaneGroupRef& connected_lg : connected_lgs) {
      if (!visited_lane_groups[connected_lg]) {
        lane_groups_to_visit.push(connected_lg);
      }
    }
  }
  return reachable_lane_groups;
}


std::vector<LaneGroupRef> MapTransverser::getAllValidLaneGroupsInMap() const
{
  /*
   * Returns all refs for all lane groups in the map which are valid
   */

  std::vector<LaneGroupRef> valid_lane_groups;
  for (const auto& tile_pair : map_.tiles_) {
    const auto& tile_id = tile_pair.first;
    for (const auto& lg_pair : tile_pair.second.lane_groups) {
      LaneGroupRef lg_ref;
      lg_ref.tile_id = tile_pair.first;
      lg_ref.id = lg_pair.first;
      if (isDirectionalLaneGroup(lg_ref)) {
        valid_lane_groups.push_back(lg_ref);
      }
    }
  }
  return valid_lane_groups;
}


std::vector<LaneGroupRef>
MapTransverser::getOutFlowingLaneGroups(const ConnectorRef& conn_ref) const
{
  std::vector<LaneGroupRef> outflowing_connectors;
  if (map_.connectorExists(conn_ref)) {
    const Connector& conn = map_.getConnectorFromRef(conn_ref);
    for (const auto& lg_ref : conn.connected_lane_groups) {
      if (isDirectionalLaneGroup(lg_ref)) {
        std::string dot = getDirectionOfTravel(lg_ref);
        const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
        if (conn_ref == lg.start_connector) {
          if (dot == "FORWARD") {
            // it's flowing out of connector
            outflowing_connectors.push_back(lg_ref);
          }
        }
        if (conn_ref == lg.end_connector) {
          if (dot == "BACKWARD") {
            // it's flowing out of connector
            outflowing_connectors.push_back(lg_ref);
          }
        }
      }
    }
  }
  return outflowing_connectors;
}


std::vector<LaneGroupRef> MapTransverser::getInFlowingLaneGroups(const ConnectorRef& conn_ref) const
{
  std::vector<LaneGroupRef> inflowing_connectors;
  if (map_.connectorExists(conn_ref)) {
    const Connector& conn = map_.getConnectorFromRef(conn_ref);
    for (const auto& lg_ref : conn.connected_lane_groups) {
      if (isDirectionalLaneGroup(lg_ref)) {
        std::string dot = getDirectionOfTravel(lg_ref);
        const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
        if (conn_ref == lg.start_connector) {
          if (dot == "BACKWARD") {
            // it's flowing out of connector
            inflowing_connectors.push_back(lg_ref);
          }
        }
        if (conn_ref == lg.end_connector) {
          if (dot == "FORWARD") {
            // it's flowing out of connector
            inflowing_connectors.push_back(lg_ref);
          }
        }
      }
    }
  }
  return inflowing_connectors;
}


std::vector<LaneRef> MapTransverser::getConnectedLanes(const ConnectorRef& conn_ref,
                                                       const int connector_lane_number,
                                                       const std::string& flow_direction) const
{
  std::vector<LaneGroupRef> lg_refs;
  bool valid_flow_direction = false;
  if (flow_direction == "OUTFLOWING") {
    valid_flow_direction = true;
    lg_refs = getOutFlowingLaneGroups(conn_ref);
  }
  if (flow_direction == "INFLOWING") {
    valid_flow_direction = true;
    lg_refs = getInFlowingLaneGroups(conn_ref);
  }
  if (flow_direction == "BOTH") {
    valid_flow_direction = true;
    lg_refs = getConnectedLaneGroups(conn_ref);
  }

  assert(valid_flow_direction);
  // Now we just figure out which of these are connected:
  std::vector<LaneRef> connected_lane_refs;
  for (const auto& lg_ref : lg_refs) {
    if (map_.laneGroupExists(lg_ref)) {
      const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
      for (auto& lane_pair : lg.lanes) {
        const Lane& lane = lane_pair.second;
        int connection_id;
        if (lg.start_connector == conn_ref) {
          connection_id = lane.start_connector_lane_number;
        } else {
          connection_id = lane.end_connector_lane_number;
        }
        if (connection_id == connector_lane_number) {
          LaneRef lane_ref = map_.getLaneRef(lg_ref, lane.id);
          connected_lane_refs.push_back(lane_ref);
        }
      }
    }
  }
  return connected_lane_refs;
}


int MapTransverser::getForwardLaneConnectorLaneNumber(const LaneRef& lane_ref) const
{
  const Lane& lane = map_.getLaneFromRef(lane_ref);
  std::string dot = lane.direction_of_travel;
  if (dot == "FORWARD") {
    return lane.end_connector_lane_number;
  } else if (dot == "BACKWARD") {
    return lane.start_connector_lane_number;
  }
  throwDataError(lane_ref, "getForwardLaneConnectorLaneNumber: ambiguous dot");
}


int MapTransverser::getBackwardLaneConnectorLaneNumber(const LaneRef& lane_ref) const
{
  const Lane& lane = map_.getLaneFromRef(lane_ref);
  std::string dot = lane.direction_of_travel;
  if (dot == "FORWARD") {
    return lane.start_connector_lane_number;
  } else if (dot == "BACKWARD") {
    return lane.end_connector_lane_number;
  }
  throwDataError(lane_ref, "getBackwardLaneConnectorLaneNumber: ambiguous dot");
}


std::vector<LaneRef> MapTransverser::getLanesInOrder(const LaneGroupRef& lg_ref) const
{
  /*
   * This function returns the lanes in the lane group such that if you are
   * looking in the direction of traffic, you'll get the left most lane
   * at beggining of vector and right most lane at the end; Returns empty
   * vector when a lane group with ambigous direction is input.
   */
  std::vector<LaneRef> lanes_in_order;
  if (isDirectionalLaneGroup(lg_ref)) {
    const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
    std::string dot = getDirectionOfTravel(lg_ref);
    std::vector<int> ordered_lane_ids = map_.getOrderedNonEmergencyLaneIds(lg_ref);
    for (const int i_lane_id : ordered_lane_ids) {
      LaneRef tmp;
      tmp.id = i_lane_id;
      tmp.lg_id = lg_ref.id;
      tmp.tile_id = lg_ref.tile_id;
      lanes_in_order.push_back(tmp);
    }
    if (dot == "BACKWARD") {
      std::reverse(lanes_in_order.begin(), lanes_in_order.end());
    }
  }
  return lanes_in_order;
}


std::vector<LaneRef> MapTransverser::getLanesToLeft(const LaneRef& lane_ref) const
{
  /*
   * Return lanes to the left of the provided lane ordered such that
   * the first lane in the vector is the one immediately to the left of the
   * provided lane and the last lane in the ref is the left most lane.
   */
  std::vector<LaneRef> lanes_to_the_left;
  LaneGroupRef lg_ref = map_.getLaneGroupRef(lane_ref);
  if (isDirectionalLaneGroup(lg_ref)) {
    std::vector<LaneRef> left_to_right_lanes;
    left_to_right_lanes = getLanesInOrder(lg_ref);
    std::string dot = getDirectionOfTravel(lg_ref);
    if (dot == "FORWARD") {
      for (const auto i_lane_ref : left_to_right_lanes) {
        if (i_lane_ref.id < lane_ref.id) {
          lanes_to_the_left.push_back(i_lane_ref);
        }
      }
    }
    if (dot == "BACKWARD") {
      for (const auto i_lane_ref : left_to_right_lanes) {
        if (i_lane_ref.id > lane_ref.id) {
          lanes_to_the_left.push_back(i_lane_ref);
        }
      }
    }
  }
  std::reverse(lanes_to_the_left.begin(), lanes_to_the_left.end());
  return lanes_to_the_left;
}


std::vector<LaneRef> MapTransverser::getLanesToRight(const LaneRef& lane_ref) const
{
  /*
   * Return lanes to the right of the provided lane ordered such that
   * the first lane in the vector is the one immediately to the right of the
   * provided lane and the last lane in the ref is the right most lane.
   */
  std::vector<LaneRef> lanes_to_the_right;
  LaneGroupRef lg_ref = map_.getLaneGroupRef(lane_ref);
  if (isDirectionalLaneGroup(lg_ref)) {
    std::vector<LaneRef> left_to_right_lanes;
    left_to_right_lanes = getLanesInOrder(lg_ref);
    std::string dot = getDirectionOfTravel(lg_ref);
    if (dot == "FORWARD") {
      for (const auto i_lane_ref : left_to_right_lanes) {
        if (i_lane_ref.id > lane_ref.id) {
          lanes_to_the_right.push_back(i_lane_ref);
        }
      }
    }
    if (dot == "BACKWARD") {
      for (const auto i_lane_ref : left_to_right_lanes) {
        if (i_lane_ref.id < lane_ref.id) {
          lanes_to_the_right.push_back(i_lane_ref);
        }
      }
    }
  }
  return lanes_to_the_right;
}


BoundaryRef MapTransverser::getRightBoundary(const LaneRef& lane_ref) const
{
  const Lane& lane = map_.getLaneFromRef(lane_ref);
  int boundary_id;
  bool valid_direction = false;
  if (lane.direction_of_travel == "FORWARD") {
    valid_direction = true;
    boundary_id = lane.right_boundary_id;
  }
  if (lane.direction_of_travel == "BACKWARD") {
    valid_direction = true;
    boundary_id = lane.left_boundary_id;
  }
  // return if valid, otherwise throw error:
  if (!valid_direction) {
    throwDataError(lane_ref, "getLeftBoundary - lane has not a valid direction!");
  }

  BoundaryRef boundary_ref;
  boundary_ref.id = boundary_id;
  boundary_ref.lg_id = lane_ref.lg_id;
  boundary_ref.tile_id = lane_ref.tile_id;
  return boundary_ref;
}


BoundaryRef MapTransverser::getLeftBoundary(const LaneRef& lane_ref) const
{
  const Lane& lane = map_.getLaneFromRef(lane_ref);
  int boundary_id;
  bool valid_direction = false;
  if (lane.direction_of_travel == "FORWARD") {
    valid_direction = true;
    boundary_id = lane.left_boundary_id;
  }
  if (lane.direction_of_travel == "BACKWARD") {
    valid_direction = true;
    boundary_id = lane.right_boundary_id;
  }
  // return if valid, otherwise throw error:
  if (!valid_direction) {
    throwDataError(lane_ref, "getLeftBoundary - lane has not a valid direction!");
  }

  BoundaryRef boundary_ref;
  boundary_ref.id = boundary_id;
  boundary_ref.lg_id = lane_ref.lg_id;
  boundary_ref.tile_id = lane_ref.tile_id;
  return boundary_ref;
}


std::vector<geometry_msgs::Point> MapTransverser::getLanePtsForward(const LaneRef& lane_ref) const
{
  std::vector<geometry_msgs::Point> pts;
  if (map_.laneExists(lane_ref)) {
    const Lane& lane = map_.getLaneFromRef(lane_ref);
    if (lane.direction_of_travel == "FORWARD") {
      pts = lane.pts;
    }
    if (lane.direction_of_travel == "BACKWARD") {
      pts = lane.pts;
      std::reverse(pts.begin(), pts.end());
    }
  }
  return pts;
}


std::vector<geometry_msgs::Point> MapTransverser::getLanePtsForward(const Lane& lane) const
{
  std::vector<geometry_msgs::Point> pts;
  if (lane.direction_of_travel == "FORWARD") {
    pts = lane.pts;
  }
  if (lane.direction_of_travel == "BACKWARD") {
    pts = lane.pts;
    std::reverse(pts.begin(), pts.end());
  }
  return pts;
}


std::vector<geometry_msgs::Point>
MapTransverser::getBoundaryPtsForward(const BoundaryRef& boundary_ref) const
{
  std::vector<geometry_msgs::Point> pts;
  LaneGroupRef lg_ref = map_.getLaneGroupRef(boundary_ref);
  std::string dot = getDirectionOfTravel(lg_ref);
  if (dot == "FORWARD") {
    const Boundary& boundary = map_.getBoundaryFromRef(boundary_ref);
    pts = boundary.pts;
  }
  if (dot == "BACKWARD") {
    const Boundary& boundary = map_.getBoundaryFromRef(boundary_ref);
    pts = boundary.pts;
    std::reverse(pts.begin(), pts.end());
  }
  return pts;
}


ConnectorRef MapTransverser::getForwardConnectorRef(const LaneGroupRef& lg_ref) const
{
  if (map_.laneGroupExists(lg_ref)) {
    const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
    std::string dot = getDirectionOfTravel(lg);
    if (dot == "FORWARD") {
      return lg.end_connector;
    } else if (dot == "BACKWARD") {
      return lg.start_connector;
    }
  }
  throwDataError(lg_ref, "getForwardConnectorRef is working with a lane group with ambigious "
                         "dot");
}


ConnectorRef MapTransverser::getBackwardConnectorRef(const LaneGroupRef& lg_ref) const
{
  if (map_.laneGroupExists(lg_ref)) {
    const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
    std::string dot = getDirectionOfTravel(lg);
    assert((dot == "FORWARD") or (dot == "BACKWARD"));
    if (dot == "FORWARD") {
      return lg.start_connector;
    } else if (dot == "BACKWARD") {
      return lg.end_connector;
    }
  }
  throwDataError(lg_ref, "getForwardConnectorRef is working with a lane group with ambigious "
                         "dot");
}


bool MapTransverser::isDirectionalLaneGroup(const LaneGroupRef& lg_ref) const
{
  if (map_.laneGroupExists(lg_ref)) {
    std::string dot = getDirectionOfTravel(lg_ref);
    if ((dot == "FORWARD") or (dot == "BACKWARD")) {
      return true;
    }
  }
  return false;
}


std::string MapTransverser::getDirectionOfTravel(const LaneGroup& lg) const
{
  std::string direction_of_travel;
  int i = 0;
  for (const auto& lane_pair : lg.lanes) {
    if (not lane_pair.second.is_emergency_lane) {
      std::string dir = lane_pair.second.direction_of_travel;
      if (i == 0) {
        direction_of_travel = dir;
      }
      if (direction_of_travel != dir) {
        return "UNKNOWN";
      }
      i++;
    }
  }
  return direction_of_travel;
}


std::string MapTransverser::getDirectionOfTravel(const LaneGroupRef& lg_ref) const
{
  if (map_.laneGroupExists(lg_ref)) {
    const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
    return getDirectionOfTravel(lg);
  };
  std::string unknown = "UNKNOWN";
  return unknown;
}


std::vector<LaneGroupRef>
MapTransverser::getNextLaneGroups(const LaneGroupRef& current_lg_ref) const
{
  std::vector<LaneGroupRef> connected_lg_refs;
  ConnectorRef next_conn_ref = getForwardConnectorRef(current_lg_ref);
  if (map_.connectorExists(next_conn_ref)) {
    const Connector& next_conn = map_.getConnectorFromRef(next_conn_ref);
    std::vector<LaneGroupRef> connected_lg_refs = next_conn.connected_lane_groups;
    for (int i = 0; i < connected_lg_refs.size(); ++i) {
      if (current_lg_ref == connected_lg_refs[i]) {
        connected_lg_refs.erase(connected_lg_refs.begin() + i);
      }
    }
  }
  return connected_lg_refs;
}


std::vector<LaneRef> MapTransverser::getNearestLanesAtLocation(double x_loc, double y_loc) const
{
  return map_.getNearestLanesAtLocation(x_loc, y_loc);
}


std::vector<LaneRef> MapTransverser::getAllUnidirectionalLaneRefsInMap() const
{
  /*
   * Visits all map and returns lane refs to all non emergency lanes in map:
   */
  std::vector<LaneRef> all_lane_refs;
  for (const auto& tile_pair : map_.tiles_) {
    const auto& tile_id = tile_pair.first;
    for (const auto& lg_pair : tile_pair.second.lane_groups) {
      const auto& lg_id = lg_pair.first;
      for (const auto& lane_pair : lg_pair.second.lanes) {
        const Lane& lane = lane_pair.second;
        if (lane.direction_of_travel == "FORWARD" || lane.direction_of_travel == "BACKWARD") {
          if (!lane.is_emergency_lane) {
            LaneRef lane_ref;
            lane_ref.tile_id = tile_id;
            lane_ref.lg_id = lg_id;
            lane_ref.id = lane.id;
            all_lane_refs.push_back(lane_ref);
          }
        }
      }
    }
  }
  return all_lane_refs;
}


bool MapTransverser::isOvertakingAllowed(const LaneRef& lane_ref) const
{
  const Lane& lane = map_.getLaneFromRef(lane_ref);
  return !lane.is_no_passing_zone;
}


bool MapTransverser::isDrivingOnRightShoulderAllowed(const LaneRef& lane_ref) const
{
  const LaneGroupRef& lg_ref = map_.getLaneGroupRef(lane_ref);
  const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
  return !lg.is_right_shoulder_undrivable;
}


BoostPolygon MapTransverser::getNonEmergencyLaneGroupPerimeter(const LaneGroupRef& lg_ref) const
{
  if (map_.laneGroupExists(lg_ref)) {
    // Extract left boundary of left most lane, and right boundary of right most lane:
    std::vector<LaneRef> in_order = getLanesInOrder(lg_ref);
    const LaneRef& l_lane_ref = in_order.front();
    const LaneRef& r_lane_ref = in_order.back();
    const BoundaryRef l_boundary_ref = getLeftBoundary(l_lane_ref);
    const BoundaryRef r_boundary_ref = getRightBoundary(r_lane_ref);
    std::vector<geometry_msgs::Point> l_side = getBoundaryPtsForward(l_boundary_ref);
    std::vector<geometry_msgs::Point> r_side = getBoundaryPtsForward(r_boundary_ref);

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
  } else {
    throwDataError(lg_ref, "Given lane group does not exist");
  }
}

std::vector<geometry_msgs::Point>
MapTransverser::getNonEmergencyLeftLaneGroupBoundary(const LaneGroupRef& lg_ref) const
{
  if (!map_.laneGroupExists(lg_ref)) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }

  // Extract left boundary of left most lane:
  std::vector<LaneRef> in_order = getLanesInOrder(lg_ref);
  const LaneRef& l_lane_ref = in_order.front();
  const BoundaryRef l_boundary_ref = getLeftBoundary(l_lane_ref);
  return getBoundaryPtsForward(l_boundary_ref);
}

std::vector<geometry_msgs::Point>
MapTransverser::getNonEmergencyRightLaneGroupBoundary(const LaneGroupRef& lg_ref) const
{
  if (!map_.laneGroupExists(lg_ref)) {
    throwDataError(lg_ref, "Given lane group does not exist");
  }
  // Extract left boundary of left most lane:
  std::vector<LaneRef> in_order = getLanesInOrder(lg_ref);
  const LaneRef& r_lane_ref = in_order.back();
  const BoundaryRef r_boundary_ref = getRightBoundary(r_lane_ref);
  return getBoundaryPtsForward(r_boundary_ref);
}

std::unordered_set<ConnectorRef>
MapTransverser::getAllConnectedConnectors(const LaneGroupRef& lg_ref) const
{
  std::unordered_set<ConnectorRef> output;
  if (map_.laneGroupExists(lg_ref)) {
    const LaneGroup& lg = map_.getLaneGroupFromRef(lg_ref);
    if (map_.connectorExists(lg.start_connector)) {
      output.insert(lg.start_connector);
    }
    if (map_.connectorExists(lg.end_connector)) {
      output.insert(lg.end_connector);
    }
  }
  return output;
}

void MapTransverser::throwDataError(const LaneRef& lane_ref, const std::string& error_msg) const
{
  std::stringstream ss;
  ss << error_msg;
  ss << ", lane_ref: " << lane_ref.tile_id << ", " << lane_ref.lg_id << ", " << lane_ref.id;
  throw MapDataError(ss.str());
}


void MapTransverser::throwDataError(const LaneGroupRef& lg_ref, const std::string& error_msg) const
{
  std::stringstream ss;
  ss << error_msg;
  ss << ", lane_group_ref: " << lg_ref.tile_id << ", " << lg_ref.id;
  throw MapDataError(ss.str());
}


} // namespace mappery
