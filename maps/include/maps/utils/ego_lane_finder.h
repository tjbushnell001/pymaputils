#ifndef LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H
#define LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H

#include <math.h>
#include <utils/map/utils.h>
#include <vector>
#include "lane_map_utils.h"

namespace ego_lane_finder {

const double MAX_DISTANCE_TO_LANE = 0.10; // meters

/**
 * Get all lanes geometrically associated with a point.
 *
 * This applies MAX_DISTANCE_TO_LANE overlap to prevent against small gaps
 * between adjacent and between successive lane segments.  This can result in
 * associations with both of otherwise non-overlapping lane segments.
 *
 * @param map The current lane map (in vehicle frame).
 * @param object Object point to associate.
 *
 * @return An unordered set of associated lanes.
 **/
std::unordered_set<lane_map::LaneRef> associateLanes(const maps::LaneSubMap& map,
                                                     const geometry_msgs::Point& object);

/**
 * Determine current ego lane.
 *
 * See notes regarding AMBIGUITY on getNominalLanes().
 * This uses route_lane_groups to only choose lane groups on our path, which
 * should remove any lane group ambiguity.  There still can be lane ambiguity
 * due to map issues or overlapping lanes.
 *
 * @param map The current lane map (in vehicle frame).
 * @param route_lane_groups Nearby allowed lane groups on the current ego route.
 *
 * @returns A vector of LaneRefs.  Sorted by likelihood of being ego lane.
 **/
std::vector<lane_map::LaneRef>
getEgoLanes(const maps::LaneSubMap& map,
            const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups);

/**
 * Determine the nominal lane to follow at a junction, which should be more or
 * less the "most acceptable" lane to stay in the current lane.
 *
 * This is a heuristic which uses a variety of lane and lane group features
 * (lane_transition_type, is_ramp, lane order, etc) to disambiguate between
 * multiple options.
 *
 * Notes regarding AMBIGUITY:
 * 1. This function uses lane_transition_type to prefer normal continuing lanes
 *    over merging or newly splitting ones.  If there are multiple of these, it
 *    prefers the left-most such lane.
 * 2. While it can decide between ramp and non/ramp lane groups, it can't help
 *    decide route level decision. It otherwise treats lane groups as equivalent
 *    and will arbitrarily prefer the lane group with the lowest id.
 * 3. Overall, multiple results means *lanes* were ambiguous.
 * 4. Further, multiple results with lanes from multiple lane groups indicates
 *    *lane group* ambiguity which means there are multiple equivalent routes
 *    that could be followed.
 * 5. If there is a clear winner, there is only one result. However, When there
 *    are multiple ambiguous lanes (ones it considers equivalent), this function
 *    sorts those in decreasing order of preference.
 *
 * @param map The current lane map (in vehicle frame).
 * @param candidate_lanes Lanes to choose from.
 * @param direction Which direction we're travelling relative to junction.
 *
 * @returns A vector of LaneRefs.  Sorted by decreasing preference.
 **/
std::vector<lane_map::LaneRef> getNominalLanes(
    const maps::LaneSubMap& map, const std::unordered_set<lane_map::LaneRef>& candidate_lanes,
    lane_map_utils::TraverseDirection direction);


} // end namespace

#endif
