#ifndef MAPS_MAP_FRAME_H_
#define MAPS_MAP_FRAME_H_

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include "utils/map/utils.h"

namespace maps {

enum class MapFrameType
{
  INVALID,
  // map is in GCS (with standard x=lon, y=lat points; aka ENU)
  GCS,
  // map is in GCS (with reversed x=lat, y=lon points; aka NED)
  GCS_NED,
  // map is in UTM (in utm_zone)
  UTM,
  // map is in vehicle frame
  VEHICLE
};

struct MapFrame
{
  /**
   * The frame of representation for a map
   **/

  MapFrameType type = MapFrameType::INVALID;

  /**
   * GCS, GCS_NED, UTM: clock time of map origin update
   * VEHICLE: timestamp of frame transform (e.g. from Localization msg)
   **/
  ros::Time timestamp;

  /**
   * GCS, GCS_NED, UTM: Rough center point sufficient to load current map view.
   *                    Do not assume this is the same as vehicle localization.
   *                    Heading may be 0.
   * VEHICLE: GCS to vehicle frame transform
   **/
  double origin_latitude;
  double origin_longitude;
  double origin_heading;

  /**
   * UTM translation of origin latitude/longitude/heading above.
   * GCS, GCS_NED, UTM: Pose may be 0.
   * UTM: All tiles are in utm_zone.
   * VEHICLE: Use utm_zone for frame transforms.
   **/
  geometry_msgs::Pose utm_origin;
  map_utils::UtmZone utm_zone;
};

}; // namespace maps

#endif // MAPS_MAP_FRAME_H_
