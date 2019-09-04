#ifndef UTILS_MISSION_MISSION_H
#define UTILS_MISSION_MISSION_H

#include <maps/map_layers.h>
#include "perception_msgs/MapTrip.h"

namespace mission_plan {

class MissionPlan
{
 public:
  /**
   * Keeps track of a current trip.
   **/
  MissionPlan(const maps::MapLayers& maps);

  void updateTrip(const perception_msgs::MapTrip& trip);

  /**
   * @returns bool indicating if a trip is active
   **/
  bool isActive() const;

  /**
   * Get nearby lane groups that are on our route.
   *
   * @returns A set of LaneGroupRefs. Empty if no route is active.
   **/
  std::unordered_set<lane_map::LaneGroupRef> getRouteLaneGroups() const;

  /**
   * Get nearby road segments that are on our route.
   *
   * @returns A set of RoadSegmentRefs. Empty if no route is active.
   **/
  std::unordered_set<road_map::RoadSegmentRef> getRouteSegments() const;

  /**
   * Get the currently active trip.
   *
   * @returns A MapTrip, or nullptr if not active.
   **/
  perception_msgs::MapTrip::ConstPtr getTrip() const;

 private:
  const maps::MapLayers& maps_;
  perception_msgs::MapTrip::ConstPtr trip_;

  mutable std::unordered_set<uint64_t> route_tiles_;
  mutable std::unordered_set<lane_map::LaneGroupRef> route_lane_groups_;
  mutable std::unordered_set<road_map::RoadSegmentRef> route_road_segs_;
};

}; // namespace mission_planning

#endif // UTILS_MISSION_MISSION_H
