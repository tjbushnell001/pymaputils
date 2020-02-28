#include "maps/utils/lane_map_following.h"

namespace lane_map_utils {

const lane_map::LaneRef
getNextRouteLane(const maps::LaneSubMap& map,
                 const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups,
                 const lane_map::LaneRef& lane_ref)
{
  const lane_map::Lane* lane = map.getLane(lane_ref);
  if (!lane) {
    return lane_map::LaneRef::INVALID;
  }
  auto end_junction = map.getJunction(lane->end_junction_ref);
  if (!end_junction) {
    return lane_map::LaneRef::INVALID;
  }
  // Get junction info from input:
  std::unordered_set<lane_map::LaneRef> outflowing_lanes;

  std::copy_if(end_junction->outflow_refs.begin(), end_junction->outflow_refs.end(),
               std::inserter(outflowing_lanes, outflowing_lanes.begin()),
               [&route_lane_groups](const lane_map::LaneRef& lane_ref) {
                 const auto lane_group_ref = lane_ref.getLaneGroupRef();
                 return route_lane_groups.count(lane_group_ref) > 0;
               });

  std::vector<lane_map::LaneRef> nominal_lanes =
      ego_lane_finder::getNominalLanes(map, outflowing_lanes, TraverseDirection::OUT);

  if (nominal_lanes.empty()) {
    return lane_map::LaneRef::INVALID;
  }
  if (!map.getLane(nominal_lanes.front())) {
    return lane_map::LaneRef::INVALID;
  }

  return nominal_lanes.front();
};

const lane_map::LaneGroupRef
getNextRouteLaneGroup(const maps::LaneSubMap& map,
                      const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups,
                      const lane_map::LaneGroupRef& lane_group_ref)
{
  const lane_map::LaneGroup* lane_group = map.getLaneGroup(lane_group_ref);
  if (!lane_group) {
    return lane_map::LaneGroupRef::INVALID;
  }
  auto end_connector = map.getConnector(lane_group->end_connector);
  if (!end_connector) {
    return lane_map::LaneGroupRef::INVALID;
  }

  std::unordered_set<lane_map::LaneGroupRef> outflowing_lanes;
  for (const auto& lane_group_ref : end_connector->outflow_refs) {
    if (route_lane_groups.count(lane_group_ref) == 0 && map.getLaneGroup(lane_group_ref)) {
      // on route and it exists
      return lane_group_ref;
    }
  }

  return lane_map::LaneGroupRef::INVALID;
}

} // namespace lane_map_utils
