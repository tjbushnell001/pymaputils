#ifndef MAPS_MAP_UTILS_LANE_MAP_FOLLOWING
#define MAPS_MAP_UTILS_LANE_MAP_FOLLOWING

#include <unordered_set>

#include "lane_map_utils.h"
#include "maps/utils/ego_lane_finder.h"

namespace lane_map_utils {

/**
 * Given a LaneRef, return the next LaneRef to traverse while following the
 * given route.
 *
 * Note: This uses a heuristic, when multiple such lanes are possible. See
 *       notes for ego_lane_finder::getNominalLanes().
 *
 * @param[in] map The relevant lane_map to follow.
 * @param[in] route_lane_groups Allowed lane groups on this route.
 * @param[in] lane_ref LaneRef of the starting lane segment.
 *
 * @return LaneRef of next lane segment, or LaneRef::INVALID if none.
 **/
const lane_map::LaneRef
getNextRouteLane(const maps::LaneSubMap& map,
                 const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups,
                 const lane_map::LaneRef& lane_ref);

/**
 * Given a LaneGroupRef, return the next LaneGroupRef to traverse while
 * following the given route.
 *
 * @param[in] map The relevant lane_map to follow.
 * @param[in] route_lane_groups Allowed lane groups on this route.
 * @param[in] lane_group_ref LaneGroupRef of the starting lane group.
 *
 * @return LaneGroupRef of next lane segment, or LaneGroupRef::INVALID if none.
 **/
const lane_map::LaneGroupRef
getNextRouteLaneGroup(const maps::LaneSubMap& map,
                      const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups,
                      const lane_map::LaneGroupRef& lane_group_ref);

}; // namespace lane_map_utils

#endif // MAPS_MAP_UTILS_LANE_MAP_FOLLOWING
