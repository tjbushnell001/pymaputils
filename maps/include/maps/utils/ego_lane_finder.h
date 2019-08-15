#ifndef LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H
#define LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H

#include <math.h>
#include <utils/map/utils.h>
#include <vector>
#include "lane_map_utils.h"

namespace ego_lane_finder {

const double MAX_DISTANCE_TO_LANE = 0.10; // meters

std::vector<lane_map::LaneRef>
getEgoLanes(const maps::LaneSubMap& map,
            const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups);

std::vector<lane_map::LaneRef>
getNominalLanes(const maps::LaneSubMap& map,
                const std::unordered_set<lane_map::LaneRef>& candidate_lanes,
                lane_map_utils::TraverseDirection direction);

std::unordered_set<lane_map::LaneRef> associateLanes(const maps::LaneSubMap& map, const geometry_msgs::Point& object);


} // end namespace

#endif
