#include <maps/mission.h>
#include <ros/ros.h>

using namespace mission_plan;

MissionPlan::MissionPlan(const maps::MapLayers& maps) : maps_(maps)
{
}

void MissionPlan::updateTrip(const perception_msgs::MapTrip& trip)
{
  if (trip_ && trip_->header.stamp == trip.header.stamp) {
    // not a new trip
    return;
  }

  ROS_WARN("Route updated. Resetting mission.");
  trip_ = boost::make_shared<const perception_msgs::MapTrip>(trip);

  route_road_segs_.clear();
  route_lane_groups_.clear();
  route_tiles_.clear();
}

bool MissionPlan::isActive() const
{
  return trip_ && !trip_->inactive;
}

perception_msgs::MapTrip::ConstPtr MissionPlan::getTrip() const
{
  if (!isActive()) {
    return nullptr;
  }
  return trip_;
}

std::unordered_set<road_map::RoadSegmentRef> MissionPlan::getRouteSegments() const
{
  return route_road_segs_;
}

std::unordered_set<lane_map::LaneGroupRef> MissionPlan::getRouteLaneGroups() const
{
  if (!isActive()) {
    return {};
  }

  auto road_map = maps_.getLayerAs<maps::RoadMapLayer>(maps::MapLayerType::ROAD)->getSubMap();
  assert(road_map);

  const auto current_route_tiles = road_map->getCurrentTileIds();
  if (current_route_tiles == route_tiles_) {
    return route_lane_groups_;
  }

  for (const auto& route : trip_->routes) {
    for (const auto& road_segment_ref : route.road_segment_refs) {
      const road_map::RoadSegment* road_segment = road_map->getRoadSegment(road_segment_ref);
      if (!road_segment)
        continue;

      route_road_segs_.insert(road_segment_ref);

      route_lane_groups_.insert(road_segment->lane_group_refs.begin(),
                                road_segment->lane_group_refs.end());
    }
  }
  route_tiles_ = current_route_tiles;

  return route_lane_groups_;
}
