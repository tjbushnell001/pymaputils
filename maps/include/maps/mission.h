#ifndef UTILS_MISSION_MISSION_H
#define UTILS_MISSION_MISSION_H

#include <maps/map_layers.h>
#include "perception_msgs/MapTrip.h"

namespace mission_plan {


class MissionPlan
{
 public:
  MissionPlan(const maps::MapLayers& maps);

  void updateTrip(perception_msgs::MapTrip::ConstPtr trip);

  bool isActive() const;

  std::unordered_set<lane_map::LaneGroupRef> getRouteLaneGroups() const;

 private:
  const maps::MapLayers& maps_;
  perception_msgs::MapTrip::ConstPtr trip_;

  mutable std::unordered_set<uint64_t> route_tiles_;
  mutable std::unordered_set<lane_map::LaneGroupRef> route_lane_groups_;
};


}; // namespace mission_planning

#endif // UTILS_MISSION_MISSION_H
