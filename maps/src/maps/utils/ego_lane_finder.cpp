#include "ego_lane_finder.h"
#include <utils/geometric/utils.h>
#include <algorithm>
#include "maps/utils/lane_map_utils.h"

namespace ego_lane_finder {

std::vector<lane_map::LaneRef>
getEgoLanes(const maps::LaneSubMap& map)
{
  /*
   * Use this function to get full functionality of ego lane finder. This function
   * uses all functions available below to:
   * 1. Find all lanes that could be ego lanes.
   * 2. Filter out lanes which are unlikely to be ego lane based on distance and
   *    angle.
   * 3. Output ego lanes ordered from left to right.
   */

  // From each lane group the ego vehicle occupies, we get the closest lane:
  std::vector<lane_map::LaneRef> candidate_lanes = getCandidateLanes(map);

  // Filter based on heading angle only if we have more than one candidate lane:
  std::vector<lane_map::LaneRef> within_angle;
  if (candidate_lanes.size() > 1) {
    within_angle = filterUsingHeading(candidate_lanes, map);
  } else {
    within_angle = candidate_lanes;
  }

  // Filter lanes that are farther away than our threshold if more than one:
  std::vector<lane_map::LaneRef> near_lanes;
  if (within_angle.size() > 1) {
    near_lanes = filterDistantLanes(within_angle, map);
  } else {
    return within_angle;
  }

  // If any lanes are still left, then just return them ordered from left to right:
  return sortFromLeftToRight(near_lanes, map);
}

  boost::optional<lane_map::LaneRef> getClosestLaneInLaneGroup(const maps::LaneSubMap& map, const lane_map::LaneGroupRef& lg_ref)
{
  const lane_map::LaneGroup* lg = map.getLaneGroup(lg_ref);
  if (!lg) {
    return {};
  }
  // find nearest non-emergency lane:
  boost::optional<lane_map::LaneRef> nearest_lane_ref;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& lane_pair : lg->lanes) {
    const lane_map::Lane& lane = lane_pair.second;
    if (!lane.is_emergency_lane) {
      const double dist = geometric::minDistanceToPiecewiseLine(0, 0, lane.pts);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_lane_ref = lane.ref;
      }
    }
  }
  return nearest_lane_ref;
}

std::vector<lane_map::LaneRef> getCandidateLanes(const maps::LaneSubMap& map)
{
  /*
   * This function finds the lane in the map which the ego vehicle is closest to.
   * The ego vehicle must be inside at least one lane group. The function also resolves
   * conflicts when the ego vehicle is inside more than one lane group (like when there
   * is an overhead bridge crossing a highway) by using the vehicle heading.
   */
  const auto candidate_lane_groups = lane_map_utils::getLaneGroupsAtLocation(map, 0.0, 0.0);

  // find the closest lane in each candidate lane group
  std::vector<lane_map::LaneRef> candidate_lanes;
  for (const auto& lg_ref : candidate_lane_groups) {
    const auto lane_ref = getClosestLaneInLaneGroup(map, lg_ref);
    if (lane_ref) {
      candidate_lanes.push_back(lane_ref.get());
    }
  }
  
  return candidate_lanes;
}


std::vector<lane_map::LaneRef> filterUsingHeading(const std::vector<lane_map::LaneRef>& lane_refs,
                                        const maps::LaneSubMap& map)
{
  std::vector<lane_map::LaneRef> within_angular_threshold;
  for (const auto& lane : lane_refs) {
    double theta = getAngleBetweenEgoHeadingAndLane(lane, map);
    if (theta <= MAX_ANGLE_DIFF)
      within_angular_threshold.push_back(lane);
  }
  return within_angular_threshold;
}


std::vector<lane_map::LaneRef> filterDistantLanes(const std::vector<lane_map::LaneRef>& lane_refs,
                                        const maps::LaneSubMap& map)
{
  /*
   * Two step filter to remove lanes based on distance metrics.
   */

  // Structure used so we can sort by distance;
  struct LaneWithDistance
  {
    lane_map::LaneRef lane_ref;
    double dist_to_ego;
  };

  // First filter: Keep lane refs which are within distance threshold:
  std::vector<LaneWithDistance> within_distance;
  for (const auto& lane : lane_refs) {
    std::vector<geometry_msgs::Point> lane_pts;
    lane_pts = lane_map_utils::getLanePtsForward(map, lane);
    double d = geometric::minDistanceToPiecewiseLine(0., 0., lane_pts);
    if (d < MAX_DISTANCE_TO_PATH) {
      LaneWithDistance lane_with_distance{ lane, d };
      within_distance.push_back(lane_with_distance);
    }
  }


  // Early return if we have 1 or less lanes left:
  if (within_distance.size() < 2) {
    std::vector<lane_map::LaneRef> to_return;
    for (const auto& lane_with_dist : within_distance) {
      to_return.push_back(lane_with_dist.lane_ref);
    }
    return to_return;
  }

  // Sort by distance to get ready for next filter:
  auto sort_fn = [](const LaneWithDistance& lhs, const LaneWithDistance& rhs) {
    return lhs.dist_to_ego < rhs.dist_to_ego;
  };
  std::sort(within_distance.begin(), within_distance.end(), sort_fn);
  std::vector<LaneWithDistance>& ordered_by_distance = within_distance;

  // Second filter: Remove any lanes which are farther away then threshold from closest lane;
  LaneWithDistance& nearest_lane = ordered_by_distance[0];
  std::vector<lane_map::LaneRef> near_lanes;
  for (const auto& lane_with_dist : ordered_by_distance) {
    double diff = std::fabs(lane_with_dist.dist_to_ego - nearest_lane.dist_to_ego);
    if (diff < MAX_DISTANCE_DIFF) {
      near_lanes.push_back(lane_with_dist.lane_ref);
    }
  }

  return near_lanes;
}


std::vector<lane_map::LaneRef> sortFromLeftToRight(const std::vector<lane_map::LaneRef>& lane_refs,
                                         const maps::LaneSubMap& map)
{
  /*
   * Uses getCompositeLateralValue to measure the leftness of lanes. It then uses
   * this to sort all input lanes from left to right.
   */
  // structure to be used for sorting:
  struct InformedLane
  {
    lane_map::LaneRef lane_ref;
    double lateral_val;
  };
  // For each lane, compute lateral value to be used to sort from left to right:
  std::vector<InformedLane> laterally_informed_lanes;
  for (const auto& lane_ref : lane_refs) {
    double lateral_val = getCompositeLateralValue(lane_ref, map);
    InformedLane temp{ lane_ref, lateral_val };
    laterally_informed_lanes.push_back(temp);
  }
  // Sort from left to right:
  auto sort_fn = [](const InformedLane& lhs, const InformedLane& rhs) {
    return lhs.lateral_val < rhs.lateral_val;
  };
  std::sort(laterally_informed_lanes.begin(), laterally_informed_lanes.end(), sort_fn);
  // Generate output:
  std::vector<lane_map::LaneRef> output;
  for (const auto& informed_lane : laterally_informed_lanes) {
    output.push_back(informed_lane.lane_ref);
  }
  return output;
}


double getAngleBetweenEgoHeadingAndLane(const lane_map::LaneRef& lane_ref,
                                        const maps::LaneSubMap& map)
{
  /*
   * This function gets the angle between the ego heading angle and the
   * given lane's path. The lane path's direction is evaluated at the
   * point closest to the ego vehicle.
   * The resulting angle is guaranteed to between -pi to pi.
   */
  geometry_msgs::Point ego_pt;
  ego_pt.x = 0.0;
  ego_pt.y = 0.0;
  geometry_msgs::Point ego_heading;
  ego_heading.x = 1.0;
  ego_heading.y = 0.0;
  std::vector<geometry_msgs::Point> lane_pts;
  lane_pts = lane_map_utils::getLanePtsForward(map, lane_ref);
  geometry_msgs::Point lane_direction;
  lane_direction = geometric::getClosestDirectionalUnitVector(ego_pt, lane_pts);
  double dtheta = geometric::angleBetweenTwoVectors(ego_heading, lane_direction);
  double dtheta_norm = geometric::normalizeAngle(dtheta);
  return dtheta_norm;
}


double getCompositeLateralValue(const lane_map::LaneRef& lane_ref, const maps::LaneSubMap& map)
{
  /*
   * This is the parameter that is used to sort lanes from left to right. A composite lateral
   * distance is calculated by computing the approximate lateral divergence both in front and
   * behind the ego vehicle. These are then added and returned.
   */
  std::vector<geometry_msgs::Point> lane_path;
  lane_path = lane_map_utils::getLanePtsForward(map, lane_ref);

  geometry_msgs::Point front_pt;
  front_pt.x = LOOK_AHEAD_DIST;
  front_pt.y = 0.;
  geometry_msgs::Point line_to_front_pt;
  line_to_front_pt = geometric::getClosestVectorFromLineToPt(front_pt, lane_path);

  geometry_msgs::Point back_pt;
  back_pt.x = -LOOK_BEHIND_DIST;
  back_pt.y = 0.;
  geometry_msgs::Point line_to_back_pt;
  line_to_back_pt = geometric::getClosestVectorFromLineToPt(back_pt, lane_path);

  double front_n_back_lat_dist = line_to_front_pt.y + line_to_back_pt.y;
  return front_n_back_lat_dist;
}


} // end namespace
