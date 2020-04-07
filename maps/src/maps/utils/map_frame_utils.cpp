#include "maps/utils/map_frame_utils.h"
#include <tf/transform_datatypes.h>
#include "utils/map/utils.h"

namespace maps {

MapFrame mapFrameFromLatLng(const MapFrameType frame_type, const double lat, const double lng,
                            const map_utils::UtmZone utm_zone)
{
  MapFrame map_frame;

  map_frame.type = frame_type;
  // NOTE: here, we use now
  map_frame.timestamp = ros::Time::now();

  map_frame.origin_latitude = lat;
  map_frame.origin_longitude = lng;

  geometry_msgs::Point utm_origin;
  utm_origin.x = lng;
  utm_origin.y = lat;
  map_utils::convertGpsToUtm(&utm_origin, utm_zone, false);
  map_frame.utm_origin.position = utm_origin;
  // NOTE: we leave utm_origin.pose and origin_heading at 0,

  map_frame.utm_zone = utm_zone;

  return map_frame;
}

MapFrame mapFrameFromLocalization(const MapFrameType frame_type,
                                  const perception_msgs::Localization& localization)
{
  MapFrame map_frame;

  map_frame.type = frame_type;
  map_frame.timestamp = localization.header.stamp;

  map_frame.origin_latitude = localization.latitude;
  map_frame.origin_longitude = localization.longitude;

  const map_utils::UtmZone utm_zone = utmZoneFromLocalization(localization);
  const double utm_theta = map_utils::getUtmTheta(utm_zone, localization.heading,
                                                  localization.latitude, localization.longitude);

  map_utils::convertGpsToUtm(localization.latitude, localization.longitude, utm_zone,
                             &map_frame.utm_origin.position.x, &map_frame.utm_origin.position.y);

  map_frame.utm_origin.orientation = tf::createQuaternionMsgFromYaw(utm_theta);

  map_frame.utm_zone = utm_zone;

  return map_frame;
}

map_utils::UtmZone utmZoneFromLocalization(const perception_msgs::Localization& localization)
{
  const map_utils::UtmZone utm_zone =
      map_utils::getUtmZone(localization.latitude, localization.longitude);

  return utm_zone;
}

MapFrame mapFrameFromMapFrameMsg(const perception_msgs::MapFrame& msg)
{
  MapFrame map_frame;

  // fully resolve frame_id, in case it is missing a leading slash
  ros::NodeHandle nh("/");
  const std::string frame_id = nh.resolveName(msg.header.frame_id);

  // setup up map frame from message.
  MapFrameType frame_type;
  if (frame_id == "/wgs84") {
    frame_type = MapFrameType::GCS;
  } else if (frame_id == "/gcs") {
    frame_type = MapFrameType::GCS_NED;
  } else if (frame_id == "/utm") {
    frame_type = MapFrameType::UTM;
  } else if (frame_id == "/vehicle_frame") {
    frame_type = MapFrameType::VEHICLE;
  } else {
    frame_type = MapFrameType::INVALID;
  }
  map_frame.type = frame_type;

  map_frame.timestamp = msg.header.stamp;

  map_frame.origin_latitude = msg.origin_latitude;
  map_frame.origin_longitude = msg.origin_longitude;

  map_frame.utm_origin = msg.origin_utm;
  map_frame.utm_zone = utmZoneFromMapFrameMsg(msg);

  return map_frame;
}

map_utils::UtmZone utmZoneFromMapFrameMsg(const perception_msgs::MapFrame& msg)
{
  const map_utils::UtmZone utm_zone(msg.utm_zone, msg.origin_latitude >= 0);

  return utm_zone;
}

perception_msgs::MapFrame mapFrameMsgFromMapFrame(const MapFrame& map_frame)
{
  perception_msgs::MapFrame msg;

  std::string frame_id;
  switch (map_frame.type) {
    case MapFrameType::GCS:
      frame_id = "/wgs84";
      break;
    case MapFrameType::GCS_NED:
      frame_id = "/gcs";
      break;
    case MapFrameType::UTM:
      frame_id = "/utm";
      break;
    case MapFrameType::VEHICLE:
      frame_id = "/vehicle_frame";
      break;
  };

  msg.header.frame_id = frame_id;
  msg.header.stamp = map_frame.timestamp;

  msg.origin_latitude = map_frame.origin_latitude;
  msg.origin_longitude = map_frame.origin_longitude;

  msg.origin_utm = map_frame.utm_origin;
  msg.utm_zone = map_frame.utm_zone.zone;

  return msg;
}

}; // end namespace maps
