#ifndef LANE_MAP_SERVER_MAPPERY_MAP_TRANSVERSER
#define LANE_MAP_SERVER_MAPPERY_MAP_TRANSVERSER

#include <perception_msgs/Map.h>
#include <string>
#include <unordered_set>
#include <vector>
#include "mappery_map.h"
#include "utils/map/utils.h"

namespace mappery {

class MapDataError : public std::runtime_error
{
 public:
  MapDataError(const std::string& what) : std::runtime_error(what)
  {
  }
};

class MapTransverser
{
 public:
  MapTransverser(const perception_msgs::Map& map_msg);

  MapTransverser(const map_utils::Map& map);

  map_utils::LaneJunction getForwardLaneJunction(const map_utils::LaneRef& lane_ref) const;

  map_utils::LaneJunction getBackwardLaneJunction(const map_utils::LaneRef& lane_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getConnectedLaneGroups(const map_utils::ConnectorRef& conn_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getConnectedLaneGroups(const map_utils::LaneGroupRef& lg_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getConnectedUnidirectionalLaneGroupsValidInMap(const map_utils::LaneGroupRef& input_lg_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getReachableLaneGroupsValidInMap(const map_utils::LaneGroupRef& input_lg_ref) const;

  std::vector<map_utils::LaneGroupRef> getAllValidLaneGroupsInMap() const;

  std::vector<map_utils::LaneGroupRef>
  getOutFlowingLaneGroups(const map_utils::ConnectorRef& conn_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getInFlowingLaneGroups(const map_utils::ConnectorRef& conn_ref) const;

  std::vector<map_utils::LaneRef>
  getConnectedLanes(const map_utils::ConnectorRef& conn_ref, int connector_lane_number,
                    const std::string& flow_direction = "BOTH") const;

  int getForwardLaneConnectorLaneNumber(const map_utils::LaneRef& lane_ref) const;

  int getBackwardLaneConnectorLaneNumber(const map_utils::LaneRef& lane_ref) const;

  std::vector<map_utils::LaneRef> getLanesInOrder(const map_utils::LaneGroupRef& lg_ref) const;

  std::vector<map_utils::LaneRef> getLanesToLeft(const map_utils::LaneRef& lane_ref) const;

  std::vector<map_utils::LaneRef> getLanesToRight(const map_utils::LaneRef& lane_ref) const;

  map_utils::BoundaryRef getRightBoundary(const map_utils::LaneRef& lane_ref) const;

  map_utils::BoundaryRef getLeftBoundary(const map_utils::LaneRef& lane_ref) const;

  std::vector<geometry_msgs::Point> getLanePtsForward(const map_utils::LaneRef& lane_ref) const;

  std::vector<geometry_msgs::Point> getLanePtsForward(const map_utils::Lane& lane) const;

  std::vector<geometry_msgs::Point>
  getBoundaryPtsForward(const map_utils::BoundaryRef& boundary_ref) const;

  map_utils::ConnectorRef getForwardConnectorRef(const map_utils::LaneGroupRef& lg_ref) const;

  map_utils::ConnectorRef getBackwardConnectorRef(const map_utils::LaneGroupRef& lg_ref) const;

  bool isDirectionalLaneGroup(const map_utils::LaneGroupRef& lg_ref) const;

  std::string getDirectionOfTravel(const map_utils::LaneGroup& lg) const;

  std::string getDirectionOfTravel(const map_utils::LaneGroupRef& lg_ref) const;

  std::vector<map_utils::LaneGroupRef>
  getNextLaneGroups(const map_utils::LaneGroupRef& current_lg_ref) const;

  std::vector<map_utils::LaneRef> getNearestLanesAtLocation(double x_loc, double y_loc) const;

  std::vector<map_utils::LaneRef> getAllUnidirectionalLaneRefsInMap() const;

  bool isOvertakingAllowed(const map_utils::LaneRef& lane_ref) const;

  bool isDrivingOnRightShoulderAllowed(const map_utils::LaneRef& lane_ref) const;

  BoostPolygon
  getNonEmergencyLaneGroupPerimeter(const map_utils::LaneGroupRef& lg_ref) const;

  BoostPolygon getLaneGroupPerimeter(const map_utils::LaneGroupRef& lg_ref) const;

  std::vector<geometry_msgs::Point>
  getNonEmergencyLeftLaneGroupBoundary(const map_utils::LaneGroupRef& lg_ref) const;

  std::vector<geometry_msgs::Point>
  getNonEmergencyRightLaneGroupBoundary(const map_utils::LaneGroupRef& lg_ref) const;


  std::unordered_set<map_utils::ConnectorRef>
  getAllConnectedConnectors(const map_utils::LaneGroupRef& lg_ref) const;

 protected:
  Map map_;

  void throwDataError(const map_utils::LaneRef& lane_ref, const std::string& error_msg) const;

  void throwDataError(const map_utils::LaneGroupRef& lg_ref, const std::string& error_msg) const;
};

} // namespace mappery

#endif
