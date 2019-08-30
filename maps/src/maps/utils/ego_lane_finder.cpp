#include "ego_lane_finder.h"
#include <utils/geometric/utils.h>
#include <utils/sets/set_utils.h>
#include <algorithm>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include "lane_map_utils.h"

namespace ego_lane_finder {

  const static std::map<lane_map::LaneTransitionType, size_t> FORWARD_TRANSITION_PRIORITY = {
    {lane_map::LaneTransitionType::UNKNOWN, 0},
    {lane_map::LaneTransitionType::MERGE, 1},
    {lane_map::LaneTransitionType::SPLIT, 2},
  };

  const static std::map<lane_map::LaneTransitionType, size_t> BACKWARD_TRANSITION_PRIORITY = {
    {lane_map::LaneTransitionType::UNKNOWN, 0},
    {lane_map::LaneTransitionType::SPLIT, 1},
    {lane_map::LaneTransitionType::MERGE, 2},
  };

std::vector<lane_map::LaneRef>
getNominalLanes(const maps::LaneSubMap& map,
                const std::unordered_set<lane_map::LaneRef>& candidate_lanes,
                const lane_map_utils::TraverseDirection direction)
{
    std::cout << "0" << std::endl << std::endl;

  if (candidate_lanes.size() > 0) {
    std::cout << "candidate lanes is not empty " << candidate_lanes.size() << std::endl;
  }

  std::cout << "size:" << candidate_lanes.size() << std::endl;

  if (candidate_lanes.empty()) {
    std::cout << "1" << std::endl << std::endl;
    return {};
  }
    std::cout << "2" << std::endl << std::endl;

  struct LanePriority {
    lane_map::LaneRef lane_ref;
    lane_map::LaneTransitionType lane_transition_type;
    int8_t lane_order;
    lane_map::LaneGroupRef lane_group_ref;
    bool is_ramp;
  };
    std::cout << "3" << std::endl << std::endl;

  const auto& transition_priority = direction == lane_map_utils::TraverseDirection::OUT ? FORWARD_TRANSITION_PRIORITY : BACKWARD_TRANSITION_PRIORITY;
    std::cout << "4" << std::endl << std::endl;

  auto priority_fn = [&map, &transition_priority](const LanePriority& a, const LanePriority& b) {
    if (a.is_ramp < b.is_ramp) {
      // all non-ramps before all ramps
      return true;
    }
    std::cout << "5" << std::endl << std::endl;

    if (transition_priority.at(a.lane_transition_type) <
        transition_priority.at(b.lane_transition_type)) {
      return true;
    }
    std::cout << "6" << std::endl << std::endl;

    if (a.lane_group_ref == b.lane_group_ref &&
        a.lane_order < b.lane_order) {
      return true;
    } else if (a.lane_group_ref.tile_id < b.lane_group_ref.tile_id) {
      return true;
    } else if (a.lane_group_ref.tile_id == b.lane_group_ref.tile_id &&
               a.lane_group_ref.id < b.lane_group_ref.id) {
      return true;
    }
    std::cout << "7" << std::endl << std::endl;

    return false;
  };

  std::cout << "8" << std::endl << std::endl;


  auto is_equivalent = [&map](const LanePriority& a, const LanePriority& b) {
    return (a.is_ramp == b.is_ramp &&
            a.lane_transition_type == b.lane_transition_type);
  };
    std::cout << "9" << std::endl << std::endl;

  std::vector<LanePriority> lane_priorities;
  for (const auto& lane_ref : candidate_lanes) {
    const auto lg_ref = lane_ref.getLaneGroupRef();
    const auto lane = map.getLane(lane_ref);
    const auto lg = map.getLaneGroup(lg_ref);
    if (!lg || !lane) {
      // lane not found
      continue;
    }
    std::cout << "10" << std::endl << std::endl;

    const LanePriority priority = {lane_ref, lane->lane_transition_type,
                             lane->lane_order, lg_ref, lg->is_ramp};
    lane_priorities.push_back(priority);
  }
  if (lane_priorities.empty()) {
    return {};
  }
    std::cout << "11" << std::endl << std::endl;

  std::sort(lane_priorities.begin(), lane_priorities.end(), priority_fn);

  const auto& top_priority = lane_priorities.front();
  std::vector<lane_map::LaneRef> results;
  for (const auto& current_priority : lane_priorities) {
    if (!is_equivalent(current_priority, top_priority)) {
      break;
    }
    results.push_back(current_priority.lane_ref);
  }
    std::cout << "12" << std::endl << std::endl;

  return results;
}

std::vector<lane_map::LaneRef>
getEgoLanes(const maps::LaneSubMap& map,
            const std::unordered_set<lane_map::LaneGroupRef>& route_lane_groups)
{
  /*
   * Use this function to get full functionality of ego lane finder. This function
   * uses all functions available below to:
   * 1. Find all lanes that could be ego lanes.
   * 2. Filter out lanes which are unlikely to be ego lane based on distance and
   *    angle.
   * 3. Output ego lanes ordered from left to right.
   */
  const geometry_msgs::Point ego;

  const auto associated_lanes = associateLanes(map, ego);

  std::unordered_set<lane_map::LaneRef> candidate_lanes;
  std::copy_if(associated_lanes.begin(), associated_lanes.end(),
               std::inserter(candidate_lanes, candidate_lanes.begin()),
               [&](const lane_map::LaneRef& ref){
                 return route_lane_groups.count(ref.getLaneGroupRef()) > 0;
               });

  const auto nominal_lanes = getNominalLanes(map, candidate_lanes, lane_map_utils::TraverseDirection::OUT);

  return nominal_lanes;
}

std::unordered_set<lane_map::LaneRef> associateLanes(const maps::LaneSubMap& map,
                                                     const geometry_msgs::Point& object)
{
  /*
   * Filter to remove lanes base.
   */
  const auto nearby_lane_groups = lane_map_utils::getLaneGroupsAtLocation(map, object.x, object.y);

  std::vector<const lane_map::Lane*> intersecting_lanes;
  std::unordered_set<lane_map::JunctionRef> forward_junctions;
  for (const lane_map::LaneGroupRef& lg_ref : nearby_lane_groups) {
    const lane_map::LaneGroup* lg = map.getLaneGroup(lg_ref);
    if (!lg) {
      // lane group not found
      continue;
    }
    const auto lanes = lane_map_utils::getLanesInOrder(*lg);
    for (const auto& lane : lanes) {
      const auto left_boundary = map.getBoundary(lane->left_boundary_ref);
      const auto right_boundary = map.getBoundary(lane->right_boundary_ref);

      if (geometric::pointIsInLane(left_boundary->pts, right_boundary->pts, object) ||
          geometric::minDistanceToPiecewiseLine(object.x, object.y, left_boundary->pts) <
              MAX_DISTANCE_TO_LANE ||
          geometric::minDistanceToPiecewiseLine(object.x, object.y, right_boundary->pts) <
              MAX_DISTANCE_TO_LANE) {
        intersecting_lanes.push_back(lane);
        forward_junctions.insert(lane->end_junction_ref);
      }
    }
  }

  // if lanes are connected to each other, take the earlier associated lane
  // this happens commonly as we're crossing lane group boundaries
  std::unordered_set<lane_map::LaneRef> associated_lanes;
  for (const auto& lane : intersecting_lanes) {
    if (forward_junctions.count(lane->start_junction_ref) == 0) {
      associated_lanes.insert(lane->ref);
    }
  }

  return associated_lanes;
}

} // end namespace
