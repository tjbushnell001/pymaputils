#ifndef MAPS_UTILS_LANE_MAP_TRANSFORM_H_
#define MAPS_UTILS_LANE_MAP_TRANSFORM_H_

#include <ros/ros.h>
#include "maps/map_layers.h"
#include "perception_msgs/Localization.h"
#include "utils/map/utils.h"

namespace maps {
void transformMapGpsToUtm(maps::LaneSubMap* map, map_utils::UtmZone utm_zone);

void transformTileGpsToUtm(lane_map::Tile* tile, map_utils::UtmZone utm_zone, const bool use_NED);

void transformMapUtmToVehicleFrame(maps::LaneSubMap* map,
                                   const perception_msgs::Localization& localization);

void transformTileUtmToVehicleFrame(lane_map::Tile* tile,
                                    const perception_msgs::Localization& localization);

void transformMapUtmToGps(maps::LaneSubMap* map, maps::MapFrameType frame_type);

void transformTileUtmToGps(lane_map::Tile* tile, map_utils::UtmZone utm_zone, const bool use_NED);

void transformMapVehicleFrameToUtm(maps::LaneSubMap* map, map_utils::UtmZone utm_zone);

void transformTileVehicleFrameToUtm(lane_map::Tile* tile, const geometry_msgs::Point& utm_pos,
                                    const double utm_theta);

/**
 * Apply transform to all points in the map.
 * @param transform
 * @param current_type
 * @param transformed_type
 * @param map
 */
void transformMap(const Eigen::Isometry2d& transform, MapFrameType transformed_type,
                  maps::LaneSubMap* map);

} // namespace maps

#endif // MAPS_UTILS_LANE_MAP_TRANSFORM_H_
