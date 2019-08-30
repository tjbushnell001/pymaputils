#ifndef MAPS_MAP_FRAME_H_
#define MAPS_MAP_FRAME_H_

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include "utils/map/utils.h"

namespace maps {

enum class MapFrameType
{
  INVALID,
  // map is in GCS (with standard x=lng, y=lat points; aka ENU)
  GCS,
  // map is in GCS (with reversed x=lat, y=lng points; aka NED)
  GCS_NED,
  // map is in UTM (in utm_zone)
  UTM,
  // map is in vehicle frame
  VEHICLE
};

static const std::map<MapFrameType, std::string> mapFrameTypeStrings = {
    {MapFrameType::INVALID, ""},
    {MapFrameType::GCS, "GCS"},
    {MapFrameType::GCS_NED, "GCS_NED"},
    {MapFrameType::UTM, "UTM"},
    {MapFrameType::VEHICLE, "VEHICLE"},
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
  double origin_latitude = 0;
  double origin_longitude = 0;
  double origin_heading = 0;

  /**
   * UTM translation of origin latitude/longitude/heading above.
   * GCS, GCS_NED, UTM: Pose may be 0.
   * UTM: All tiles are in utm_zone.
   * VEHICLE: Use utm_zone for frame transforms.
   **/
  geometry_msgs::Pose utm_origin;
  map_utils::UtmZone utm_zone;
};

 static bool operator==(const MapFrame& frame1, const MapFrame& frame2) {
   if (frame1.type != frame2.type) {
     return false;
   }

   switch (frame1.type) {
   case MapFrameType::GCS:
   case MapFrameType::GCS_NED:
      return true;
   case MapFrameType::UTM:
     // UTM must have matching zones
     return frame1.utm_zone == frame2.utm_zone;
   case MapFrameType::VEHICLE:
     // vehicle frame must have matching localization
     return (frame1.origin_latitude == frame2.origin_latitude &&
           frame1.origin_longitude == frame2.origin_longitude &&
           frame1.origin_heading == frame2.origin_heading);
   }
   return false;
 }

  static bool operator!=(const MapFrame& frame1, const MapFrame& frame2) {
    return !(frame1 == frame2);
  }

  static bool operator<(const MapFrame& frame1, const MapFrame& frame2) {
    if (frame1.type != frame2.type) {
      return frame1.type < frame2.type;
    }

   switch (frame1.type) {
   case MapFrameType::GCS:
   case MapFrameType::GCS_NED:   
      return false;
   case MapFrameType::UTM:
     return frame1.utm_zone.zone < frame2.utm_zone.zone;
   case MapFrameType::VEHICLE:
     if (frame1.origin_latitude < frame2.origin_latitude)
       return true;
     if (frame1.origin_latitude == frame2.origin_latitude &&
         frame1.origin_longitude < frame2.origin_longitude)
       return true;
     if (frame1.origin_latitude == frame2.origin_latitude &&
         frame1.origin_longitude == frame2.origin_longitude &&
         frame1.origin_heading < frame2.origin_heading)
       return true;
     break;
   }
   return false;
 }
}; // namespace maps

#endif // MAPS_MAP_FRAME_H_
