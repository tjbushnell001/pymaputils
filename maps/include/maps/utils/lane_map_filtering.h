#ifndef MAPS_UTILS_LANE_MAP_FILTERING_H
#define MAPS_UTILS_LANE_MAP_FILTERING_H

#include <maps/lane_maps.h>
#include <maps/utils/lane_map_utils.h>
#include <utils/map/lane_map_structs.h>
#include <functional>
#include <unordered_set>

namespace lane_map_utils {
typedef std::function<bool(const lane_map::LaneGroup&)> LaneGroupFilter;
typedef std::function<bool(const lane_map::Lane&)> LaneFilter;
typedef std::function<lane_map::LaneRef(const lane_map::LaneRef&)> LaneFollower;

/**
 * Find all lane groups matching the given predicate.
 *
 * @param[in] map Lane sub map.
 * @param[in] filter_fn Predicate to filter lane groups with.
 * @returns Lane group references which match predicate.
 **/
std::unordered_set<lane_map::LaneGroupRef> filterLaneGroups(const maps::LaneSubMap& map,
                                                            const LaneGroupFilter filter_fn);

/**
 * Find all lane matching the given predicate.
 *
 * @param[in] map Lane sub map.
 * @param[in] filter_fn Predicate to filter lanes with.
 * @returns Lane references which match predicate.
 **/
std::unordered_set<lane_map::LaneRef> filterLanes(const maps::LaneSubMap& map,
                                                  const LaneFilter filter_fn);

/**
 * Find all lane groups reachable from an initial lane group, and which match
 * the given predicate.
 *
 * @param[in] map Lane sub map.
 * @param[in] initial_lg_ref Lane group to start with.
 * @param[in] filter_fn Predicate to filter lane groups with.
 * @param[in] traverse_direction Control traversal direction (forward, backward,
 *            both).
 * @param[in] max_distance Skip lane groups further than this number of steps
 *            from initial lane group. Use 0 for no limit.
 * @returns Lane group references which match predicate.
 **/
std::unordered_set<lane_map::LaneGroupRef> traverseLaneGroups(
    const maps::LaneSubMap& map, const lane_map::LaneGroupRef& initial_lg_ref,
    const LaneGroupFilter filter_fn, TraverseDirection traverse_direction, size_t max_distance);

/**
 * Follow lanes in a single direction.
 *
 * @param[in] map Lane sub map.
 * @param[in] initial_ref Starting LaneRef.
 * @param[in] follower Follower function to get next lane.
 * @param[in] limit Maximum number of steps to traverse.  Use 0 for no limit.
 * @returns A vector of lane refs.
 **/
std::vector<lane_map::LaneRef> followLanes(const lane_map::LaneRef& initial_ref,
                                           const LaneFollower& follower, size_t max_distance);

}; // namespace lane_map_utils

#endif // MAPS_UTILS_LANE_MAP_FILTERING_H
