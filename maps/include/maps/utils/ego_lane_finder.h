#ifndef LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H
#define LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H

#include <math.h>
#include <utils/map/utils.h>
#include <vector>
#include <maps/lane_maps.h>

namespace ego_lane_finder {

const double MAX_DISTANCE_TO_PATH = 2.0;           // meters
const double MAX_ANGLE_DIFF = 20.0 * M_PI / 180.0; // rad
const double MAX_DISTANCE_DIFF = 1.0;              // meters
const double LOOK_AHEAD_DIST = 10.0;               // meters
const double LOOK_BEHIND_DIST = 10.0;              // meters

std::vector<lane_map::LaneRef> getEgoLanes(const maps::LaneSubMap& lane);

std::vector<lane_map::LaneRef> getCandidateLanes(const maps::LaneSubMap& lane);

std::vector<lane_map::LaneRef> filterUsingHeading(const std::vector<lane_map::LaneRef>& lane_refs,
                                                   const maps::LaneSubMap& lane);

std::vector<lane_map::LaneRef> filterDistantLanes(const std::vector<lane_map::LaneRef>& lane_refs,
                                                   const maps::LaneSubMap& lane);

std::vector<lane_map::LaneRef>
sortFromLeftToRight(const std::vector<lane_map::LaneRef>& lane_refs,
                    const maps::LaneSubMap& lane);

double getAngleBetweenEgoHeadingAndLane(const lane_map::LaneRef& lane_ref,
                                        const maps::LaneSubMap& lane);

double getCompositeLateralValue(const lane_map::LaneRef& lane_ref,
                                const maps::LaneSubMap& lane);

} // end namespace

#endif
