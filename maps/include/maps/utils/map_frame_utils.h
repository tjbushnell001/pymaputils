#ifndef MAPS_UTILS_MAP_FRAME_UTILS_H_
#define MAPS_UTILS_MAP_FRAME_UTILS_H_

#include <ros/ros.h>
#include "maps/map_frame.h"
#include "perception_msgs/Localization.h"
#include "perception_msgs/MapFrame.h"

namespace maps {

/**
 * Create a MapFrame from lat, lng.
 *
 * NOTE: This is insufficient for VEHICLE map frames, which should use
 *       mapFrameFromLocalization().
 *
 * @param[in] lat latitude
 * @param[in] lng longitude
 * @param[in] utm_zone UTM zone to use for utm transforms.
 * @returns map frame
 **/
MapFrame mapFrameFromLatLng(const MapFrameType frame_type, const double lat, const double lng,
                            const map_utils::UtmZone utm_zone);

/**
 * Create a MapFrame from a localization message.
 *
 * @param[in] localization Localization message.
 * @returns map frame
 **/
MapFrame mapFrameFromLocalization(const MapFrameType frame_type,
                                  const perception_msgs::Localization& localization);

/**
 * Create a UtmZone from a localization message.
 *
 * @param[in] localization Localization message.
 * @returns utm zone
 **/
map_utils::UtmZone utmZoneFromLocalization(const perception_msgs::Localization& localization);

/**
 * Converts a map frame message to a MapFrame.
 *
 * @param[in] msg Map frame message.
 * @returns map frame
 **/
MapFrame mapFrameFromMapFrameMsg(const perception_msgs::MapFrame& msg);

/**

 * Create a UtmZone from a map frame message;
 *
 * @param[in] msg Map frame message.
 * @returns utm zone
 **/
map_utils::UtmZone utmZoneFromMapFrameMsg(const perception_msgs::MapFrame& msg);

/**
 * Converts a MapFrame to a map frame message.
 *
 * @param[in] msg_frame Map frame.
 * @returns map frame message.
 **/
perception_msgs::MapFrame mapFrameMsgFromMapFrame(const MapFrame& map_frame);

}; // namespace maps

#endif // MAPS_UTILS_MAP_FRAME_UTILS_H_
