#ifndef MAPS_MAP_UTILS_LANE_MAP_UTILS
#define MAPS_MAP_UTILS_LANE_MAP_UTILS

#include <string>
#include <unordered_set>
#include <vector>
#include "maps/map_layers.h"
#include "utils/map/lane_map_geometry.h"
#include "utils/map/lane_map_structs.h"
#include "utils/map/utils.h"

namespace lane_map_utils {

class MapDataError : public std::runtime_error
{
 public:
  MapDataError(const std::string& what) : std::runtime_error(what)
  {
  }
};

enum class FlowDirection
{
  IN,
  OUT,
  BOTH
};

lane_map::LaneRef createLaneRefFromLaneGroupRef(const lane_map::LaneGroupRef& lg_ref,
                                                uint8_t lane_id);

lane_map::LaneJunction getForwardLaneJunction(const maps::LaneSubMap& map,
                                              const lane_map::LaneRef& lane_ref);

lane_map::LaneJunction getBackwardLaneJunction(const maps::LaneSubMap& map,
                                               const lane_map::LaneRef& lane_ref);

/*
 * Get all lane groups which are connected to the current lane group via either
 * of its connectors but only if they are in the current map.
 */
std::unordered_set<lane_map::LaneGroupRef>
getConnectedLaneGroups(const maps::LaneSubMap& map, const lane_map::LaneGroup& lane_group,
                       FlowDirection flow_direction);

/**
 * Using an input lane group, it returns all lane groups which can be reached by visiting
 * the connector-lane group graph from this connected (in any direction).
 * It only returns lane groups which exist in the current map_
 **/
std::unordered_set<lane_map::LaneGroupRef>
getReachableValidLaneGroups(const maps::LaneSubMap& map, const lane_map::LaneGroupRef& input_lg_ref,
                            size_t max_distance = 0);

std::unordered_set<lane_map::LaneGroupRef> getAllValidLaneGroups(const maps::LaneSubMap& map);

std::unordered_set<lane_map::LaneGroupRef>
getConnectedLaneGroups(const maps::LaneSubMap& map, const lane_map::ConnectorRef& conn_ref,
                       FlowDirection flow_direction = FlowDirection::BOTH);

std::vector<lane_map::LaneRef>
getConnectedLanes(const maps::LaneSubMap& map, const lane_map::JunctionRef& conn_ref,
                  FlowDirection flow_direction = FlowDirection::BOTH);

/*
 * This function returns the lanes in the lane group such that if you are
 * looking in the direction of traffic, you'll get the left most lane
 * at beggining of vector and right most lane at the end; Returns empty
 * vector when a lane group with ambigous direction is input.
 */
std::vector<const lane_map::Lane*> getLanesInOrder(const lane_map::LaneGroup& lg);

/*
 * Return lanes to the left of the provided lane ordered such that
 * the first lane in the vector is the one immediately to the left of the
 * provided lane and the last lane in the ref is the left most lane.
 */
std::vector<lane_map::LaneRef> getLanesToLeft(const maps::LaneSubMap& map,
                                              const lane_map::LaneRef& lane_ref);

/*
 * Return lanes to the right of the provided lane ordered such that
 * the first lane in the vector is the one immediately to the right of the
 * provided lane and the last lane in the ref is the right most lane.
 */
std::vector<lane_map::LaneRef> getLanesToRight(const maps::LaneSubMap& map,
                                               const lane_map::LaneRef& lane_ref);

lane_map::BoundaryRef getRightBoundary(const lane_map::Lane& lane);

lane_map::BoundaryRef getLeftBoundary(const lane_map::Lane& lane);

std::vector<geometry_msgs::Point> getLanePtsForward(const maps::LaneSubMap& map,
                                                    const lane_map::LaneRef& lane_ref);

std::vector<geometry_msgs::Point> getLanePtsForward(const lane_map::Lane& lane);

std::vector<geometry_msgs::Point> getBoundaryPtsForward(const maps::LaneSubMap& map,
                                                        const lane_map::Boundary& boundary);

/**
 * A lane group is unidirection if all non-emergency lanes within it are in the same direction
 **/
bool isDirectionalLaneGroup(const lane_map::LaneGroup& lg);

bool isDirectionalLane(const lane_map::Lane& lane);

std::unordered_set<lane_map::LaneGroupRef> getLaneGroupsAtLocation(const maps::LaneSubMap& map,
                                                                   double x_loc, double y_loc);

/*
 * Visits all map and returns lane refs to all non emergency lanes in map:
 */
std::unordered_set<lane_map::LaneRef> getAllValidLanes(const maps::LaneSubMap& map);

bool isOvertakingAllowed(const maps::LaneSubMap& map, const lane_map::LaneRef& lane_ref);

bool isDrivingOnRightShoulderAllowed(const maps::LaneSubMap& map,
                                     const lane_map::LaneRef& lane_ref);

BoostPolygon getNonEmergencyLaneGroupPerimeter(const maps::LaneSubMap& map,
                                               const lane_map::LaneGroupRef& lg_ref);

BoostPolygon getLaneGroupPerimeter(const maps::LaneSubMap& map,
                                   const lane_map::LaneGroupRef& lg_ref);

std::vector<geometry_msgs::Point> getNonEmergencyLeftLaneGroupBoundary(
    const maps::LaneSubMap& map, const lane_map::LaneGroupRef& lg_ref);

std::vector<geometry_msgs::Point> getNonEmergencyRightLaneGroupBoundary(
    const maps::LaneSubMap& map, const lane_map::LaneGroupRef& lg_ref);

std::unordered_set<lane_map::ConnectorRef>
getAllConnectedConnectors(const maps::LaneSubMap& map, const lane_map::LaneGroupRef& lg_ref);

std::unordered_set<lane_map::LaneGroupRef> getLaneGroupsInZone(const maps::LaneSubMap& map,
                                                               const BoostPolygon& zone);

std::vector<lane_map::BoundaryRef> getBoundariesInZone(const maps::LaneSubMap& map,
                                                       const std::vector<BoostPoint>& zone);

size_t getNumberOfLanesInLaneGroup(const lane_map::LaneGroup& lg);

std::shared_ptr<maps::LaneSubMap> copySubMap(
    const maps::LaneSubMap& map, const std::unordered_set<lane_map::LaneGroupRef>& lane_groups);

// Extracts an even smaller subsection of the submap given by the
// radius around the coordinate provided
std::unordered_set<lane_map::LaneGroupRef> getNearbyValidLaneGroupsGCS(const maps::LaneSubMap& map,
                                                                       double lat, double lng,
                                                                       double max_distance);

// Extracts an even smaller subsection of the submap given by the
// radius around the coordinate provided
std::unordered_set<lane_map::LaneGroupRef> getNearbyValidLaneGroups(const maps::LaneSubMap& map,
                                                                    const geometry_msgs::Point& pt,
                                                                    double max_distance);

void throwDataError(const lane_map::LaneRef& lane_ref, const std::string& error_msg);

void throwDataError(const lane_map::LaneGroupRef& lg_ref, const std::string& error_msg);

}; // namespace lane_map_utils

#endif // MAPS_MAP_UTILS_LANE_MAP_UTILS
