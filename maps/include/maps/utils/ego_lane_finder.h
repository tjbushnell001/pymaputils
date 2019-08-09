#ifndef LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H
#define LANE_MAP_SERVER_MAPPERY_EGO_LANE_FINDER_H

#include <math.h>
#include <utils/map/utils.h>
#include <vector>
#include "map_interpreter.h"

namespace ego_lane_finder {

const double MAX_DISTANCE_TO_PATH = 2.0;           // meters
const double MAX_ANGLE_DIFF = 20.0 * M_PI / 180.0; // rad
const double MAX_DISTANCE_DIFF = 1.0;              // meters
const double LOOK_AHEAD_DIST = 10.0;               // meters
const double LOOK_BEHIND_DIST = 10.0;              // meters

std::vector<map_utils::LaneRef> getEgoLanes(const mappery::MapInterpreter& map_interpreter);

std::vector<map_utils::LaneRef> getCandidateLanes(const mappery::MapInterpreter& map_interpreter);

std::vector<map_utils::LaneRef> filterUsingHeading(const std::vector<map_utils::LaneRef>& lane_refs,
                                                   const mappery::MapInterpreter& map_interpreter);

std::vector<map_utils::LaneRef> filterDistantLanes(const std::vector<map_utils::LaneRef>& lane_refs,
                                                   const mappery::MapInterpreter& map_interpreter);

std::vector<map_utils::LaneRef>
sortFromLeftToRight(const std::vector<map_utils::LaneRef>& lane_refs,
                    const mappery::MapInterpreter& map_interpreter);

double getAngleBetweenEgoHeadingAndLane(const map_utils::LaneRef& lane_ref,
                                        const mappery::MapInterpreter& map_interpreter);

double getCompositeLateralValue(const map_utils::LaneRef& lane_ref,
                                const mappery::MapInterpreter& map_interpreter);

} // end namespace

#endif
