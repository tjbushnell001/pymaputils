#include <maps/utils/lane_map_filtering.h>

using namespace lane_map_utils;

std::unordered_set<lane_map::LaneGroupRef>
lane_map_utils::filterLaneGroups(const maps::LaneSubMap& map, const LaneGroupFilter filter_fn)
{
  std::unordered_set<lane_map::LaneGroupRef> lane_groups;

  for (const auto& tile : map.tiles) {
    for (const auto& lg : tile.second->lane_groups) {
      if (filter_fn(lg.second)) {
        lane_groups.insert(lg.first);
      }
    }
  }
  return lane_groups;
}

std::unordered_set<lane_map::LaneRef> lane_map_utils::filterLanes(const maps::LaneSubMap& map,
                                                                  const LaneFilter filter_fn)
{
  std::unordered_set<lane_map::LaneRef> lanes;

  for (const auto& tile : map.tiles) {
    for (const auto& lg_pair : tile.second->lane_groups) {
      const auto& lg = lg_pair.second;
      for (const auto& lane_pair : lg.lanes) {
        const auto& lane = lane_pair.second;
        if (filter_fn(lane)) {
          lanes.insert(lane.ref);
        }
      }
    }
  }
  return lanes;
}

std::unordered_set<lane_map::LaneGroupRef> lane_map_utils::traverseLaneGroups(
    const maps::LaneSubMap& map, const lane_map::LaneGroupRef& initial_lg_ref,
    const LaneGroupFilter filter_fn, TraverseDirection traverse_direction, size_t max_distance)
{
  std::unordered_set<lane_map::LaneGroupRef> visited_lane_groups;
  std::vector<size_t> lane_group_dist;
  std::vector<lane_map::LaneGroupRef> lane_groups_to_visit;

  if (map.getLaneGroup(initial_lg_ref)) {
    lane_groups_to_visit.push_back(initial_lg_ref);
    lane_group_dist.push_back(0);
    visited_lane_groups.insert(initial_lg_ref);
  }

  // recursively add lane groups which are connected
  for (size_t current = 0; current < lane_groups_to_visit.size(); ++current) {
    const lane_map::LaneGroupRef& lg_ref = lane_groups_to_visit[current];
    const size_t lg_dist = lane_group_dist[current];

    if (max_distance > 0 && lg_dist > max_distance) {
      // reached max recursion
      continue;
    }
    const auto lg = map.getLaneGroup(lg_ref);
    if (!lg) {
      // lane group does not exist
      continue;
    }

    // Only proceed if the lane group exists and it has not been visited.
    const auto connected_lgs = getConnectedLaneGroups(map, *lg, traverse_direction);
    for (const auto& connected_lg_ref : connected_lgs) {
      if (visited_lane_groups.count(connected_lg_ref) == 0) {
        const auto connected_lg = map.getLaneGroup(connected_lg_ref);
        if (!connected_lg || !filter_fn(*connected_lg)) {
          continue;
        }
        lane_groups_to_visit.push_back(connected_lg_ref);
        lane_group_dist.push_back(lg_dist + 1);
        visited_lane_groups.insert(connected_lg_ref);
      }
    }
  }
  return visited_lane_groups;
}

std::vector<lane_map::LaneRef> lane_map_utils::followLanes(const lane_map::LaneRef& initial_ref,
                                                           const LaneFollower& follower,
                                                           size_t max_distance)
{
  std::vector<lane_map::LaneRef> results;
  lane_map::LaneRef current_ref = initial_ref;
  while (max_distance == 0 || results.size() < max_distance) {
    results.push_back(current_ref);
    const lane_map::LaneRef* next_ref = follower(current_ref);
    if (!next_ref)
      break;

    current_ref = *next_ref;
  }

  return results;
}
