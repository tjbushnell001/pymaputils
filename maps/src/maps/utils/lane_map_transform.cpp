#include "maps/utils/lane_map_transform.h"
#include <tf/transform_datatypes.h>

using namespace maps;

void maps::transformMapGpsToUtm(maps::LaneSubMap* map, map_utils::UtmZone utm_zone)
{
  assert(map);
  auto& map_frame = map->map_frame;
  assert(map_frame.type == MapFrameType::GCS || map_frame.type == MapFrameType::GCS_NED);

  const bool use_NED = map_frame.type == maps::MapFrameType::GCS_NED;

  for (auto& tile_pair : map->tiles) {
    auto& tile = tile_pair.second;
    transformTileGpsToUtm(tile.get(), utm_zone, use_NED);
  }

  // it's now utm!
  map_frame.type = MapFrameType::UTM;
  map_frame.utm_zone = utm_zone;
}

void maps::transformTileGpsToUtm(lane_map::Tile* tile, map_utils::UtmZone utm_zone,
                                 const bool use_NED)
{
  for (auto& lg_pair : tile->lane_groups) {
    auto& lg = lg_pair.second;

    map_utils::convertGpsToUtm(&lg.left_boundary, utm_zone, use_NED);
    map_utils::convertGpsToUtm(&lg.right_boundary, utm_zone, use_NED);

    map_utils::convertGpsToUtm(&lg.perimeter, utm_zone, use_NED);
    for (auto& lane_pair : lg.lanes) {
      auto& lane = lane_pair.second;
      map_utils::convertGpsToUtm(&lane.pts, utm_zone, use_NED);
    }
    for (auto& boundary_pair : lg.boundaries) {
      auto& boundary = boundary_pair.second;
      map_utils::convertGpsToUtm(&boundary.pts, utm_zone, use_NED);
    }
  }
  for (auto& connector_pair : tile->connectors) {
    auto& connector = connector_pair.second;
    map_utils::convertGpsToUtm(&connector.boundary_geometry, utm_zone, use_NED);

    for (auto& junction : connector.junctions) {
      map_utils::convertGpsToUtm(&junction.pt, utm_zone, use_NED);
    }
  }
}

void applyTransform(const Eigen::Isometry2d& transform, BoostPolygon* polygon)
{
  Eigen::Vector2d p_mem;
  auto apply_to_pnt = [&p_mem, &transform](BoostPoint& p) {
    p_mem << p.x(), p.y();
    p_mem = transform * p_mem;
    p.x(p_mem(0));
    p.y(p_mem(1));
  };
  boost::geometry::for_each_point(*polygon, apply_to_pnt);
}

template <typename PointType>
void applyTransform(const Eigen::Isometry2d& transform, std::vector<PointType>* points)
{
  Eigen::Vector2d p_mem;
  for (auto it = points->begin(); it != points->end(); ++it) {
    p_mem << it->x, it->y;
    p_mem = transform * p_mem;
    it->x = p_mem(0);
    it->y = p_mem(1);
  }
}

template <typename PointType>
void applyTransform(const Eigen::Isometry2d& transform, PointType* point)
{
  Eigen::Vector2d p_mem(point->x, point->y);
  p_mem = transform * p_mem;
  point->x = p_mem(0);
  point->y = p_mem(1);
}

void maps::transformMap(const Eigen::Isometry2d& transform, MapFrameType transformed_type,
                          maps::LaneSubMap* map)
{
  assert(map);
  const auto& map_frame = map->map_frame;
  assert(map_frame.type != MapFrameType::GCS && map_frame.type != MapFrameType::GCS_NED);
  assert(transformed_type != MapFrameType::GCS && map_frame.type != MapFrameType::GCS_NED);

  for (auto& tile_pair : map->tiles) {
    auto& tile = tile_pair.second;
    for (auto& lg_pair : tile->lane_groups) {
      auto& lg = lg_pair.second;

      applyTransform(transform, &lg.left_boundary);
      applyTransform(transform, &lg.right_boundary);
      applyTransform(transform, &lg.perimeter);

      for (auto& lane_pair : lg.lanes) {
        auto& lane = lane_pair.second;
        applyTransform(transform, &lane.pts);
      }
      for (auto& boundary_pair : lg.boundaries) {
        auto& boundary = boundary_pair.second;
        applyTransform(transform, &boundary.pts);
      }
    }
    for (auto& connector_pair : tile->connectors) {
      auto& connector = connector_pair.second;
      applyTransform(transform, &connector.boundary_geometry);
      for (auto& junction : connector.junctions) {
        applyTransform(transform, &junction.pt);
      }
    }
  }
}

void maps::transformMapUtmToVehicleFrame(maps::LaneSubMap* map,
                                         const perception_msgs::Localization& localization)
{
  assert(map);
  const auto& map_frame = map->map_frame;
  assert(map_frame.type == MapFrameType::UTM);

  const map_utils::UtmZone utm_zone = utmZoneFromLocalization(localization);
  assert(map_frame.utm_zone == utm_zone);

  for (auto& tile_pair : map->tiles) {
    auto& tile = tile_pair.second;
    transformTileUtmToVehicleFrame(tile.get(), localization);
  }

  // it's now in vehicle frame!
  map->map_frame = mapFrameFromLocalization(MapFrameType::VEHICLE, localization);
}

void maps::transformTileUtmToVehicleFrame(lane_map::Tile* tile,
                                          const perception_msgs::Localization& localization)
{
  const map_utils::UtmZone utm_zone = utmZoneFromLocalization(localization);

  geometry_msgs::Point utm_pos;
  map_utils::convertGpsToUtm(localization.latitude, localization.longitude, utm_zone, &utm_pos.x,
                             &utm_pos.y);
  const double utm_theta = map_utils::getUtmTheta(utm_zone, localization.heading,
                                                  localization.latitude, localization.longitude);

  for (auto& lg_pair : tile->lane_groups) {
    auto& lg = lg_pair.second;

    map_utils::convertUtmToVehicleFrame(&lg.left_boundary, utm_pos.x, utm_pos.y, utm_theta);
    map_utils::convertUtmToVehicleFrame(&lg.right_boundary, utm_pos.x, utm_pos.y, utm_theta);

    map_utils::convertUtmToVehicleFrame(&lg.perimeter, utm_pos.x, utm_pos.y, utm_theta);
    for (auto& lane_pair : lg.lanes) {
      auto& lane = lane_pair.second;
      map_utils::convertUtmToVehicleFrame(&lane.pts, utm_pos.x, utm_pos.y, utm_theta);
    }
    for (auto& boundary_pair : lg.boundaries) {
      auto& boundary = boundary_pair.second;
      map_utils::convertUtmToVehicleFrame(&boundary.pts, utm_pos.x, utm_pos.y, utm_theta);
    }
  }
  for (auto& connector_pair : tile->connectors) {
    auto& connector = connector_pair.second;
    map_utils::convertUtmToVehicleFrame(&connector.boundary_geometry, utm_pos.x, utm_pos.y,
                                        utm_theta);

    for (auto& junction : connector.junctions) {
      map_utils::convertUtmToVehicleFrame(&junction.pt, utm_pos.x, utm_pos.y, utm_theta);
    }
  }
}

void maps::transformMapUtmToGps(maps::LaneSubMap* map, maps::MapFrameType frame_type)
{
  assert(map);
  assert(frame_type == MapFrameType::GCS || frame_type == MapFrameType::GCS_NED);

  auto& map_frame = map->map_frame;
  assert(map_frame.type == MapFrameType::UTM);

  const bool use_NED = frame_type == maps::MapFrameType::GCS_NED;
  for (auto& tile_pair : map->tiles) {
    auto& tile = tile_pair.second;
    transformTileUtmToGps(tile.get(), map->map_frame.utm_zone, use_NED);
  }

  // it's now gps!
  map_frame.type = MapFrameType::GCS_NED;
}

void maps::transformTileUtmToGps(lane_map::Tile* tile, map_utils::UtmZone utm_zone,
                                 const bool use_NED)
{
  for (auto& lg_pair : tile->lane_groups) {
    auto& lg = lg_pair.second;

    map_utils::convertUtmToGps(&lg.left_boundary, utm_zone, use_NED);
    map_utils::convertUtmToGps(&lg.right_boundary, utm_zone, use_NED);

    map_utils::convertUtmToGps(&lg.perimeter, utm_zone, use_NED);
    for (auto& lane_pair : lg.lanes) {
      auto& lane = lane_pair.second;
      map_utils::convertUtmToGps(&lane.pts, utm_zone, use_NED);
    }
    for (auto& boundary_pair : lg.boundaries) {
      auto& boundary = boundary_pair.second;
      map_utils::convertUtmToGps(&boundary.pts, utm_zone, use_NED);
    }
  }
  for (auto& connector_pair : tile->connectors) {
    auto& connector = connector_pair.second;
    map_utils::convertUtmToGps(&connector.boundary_geometry, utm_zone, use_NED);

    for (auto& junction : connector.junctions) {
      map_utils::convertUtmToGps(&junction.pt, utm_zone, use_NED);
    }
  }
}

void maps::transformMapVehicleFrameToUtm(maps::LaneSubMap* map, map_utils::UtmZone utm_zone)
{
  assert(map);
  auto& map_frame = map->map_frame;
  assert(map_frame.type == MapFrameType::VEHICLE);

  const auto utm_pos = map_frame.utm_origin.position;
  const double utm_theta = tf::getYaw(map_frame.utm_origin.orientation);

  for (auto& tile_pair : map->tiles) {
    auto& tile = tile_pair.second;
    transformTileVehicleFrameToUtm(tile.get(), utm_pos, utm_theta);
  }

  // it's now in utm!
  map_frame.type = MapFrameType::UTM;
  map_frame.utm_zone = utm_zone;
}

void maps::transformTileVehicleFrameToUtm(lane_map::Tile* tile, const geometry_msgs::Point& utm_pos,
                                          const double utm_theta)
{
  for (auto& lg_pair : tile->lane_groups) {
    auto& lg = lg_pair.second;

    map_utils::convertVehicleFrameToUtm(&lg.left_boundary, utm_pos.x, utm_pos.y, utm_theta);
    map_utils::convertVehicleFrameToUtm(&lg.right_boundary, utm_pos.x, utm_pos.y, utm_theta);

    map_utils::convertVehicleFrameToUtm(&lg.perimeter, utm_pos.x, utm_pos.y, utm_theta);
    for (auto& lane_pair : lg.lanes) {
      auto& lane = lane_pair.second;
      map_utils::convertVehicleFrameToUtm(&lane.pts, utm_pos.x, utm_pos.y, utm_theta);
    }
    for (auto& boundary_pair : lg.boundaries) {
      auto& boundary = boundary_pair.second;
      map_utils::convertVehicleFrameToUtm(&boundary.pts, utm_pos.x, utm_pos.y, utm_theta);
    }
  }
  for (auto& connector_pair : tile->connectors) {
    auto& connector = connector_pair.second;
    map_utils::convertUtmToVehicleFrame(&connector.boundary_geometry, utm_pos.x, utm_pos.y,
                                        utm_theta);

    for (auto& junction : connector.junctions) {
      map_utils::convertUtmToVehicleFrame(&junction.pt, utm_pos.x, utm_pos.y, utm_theta);
    }
  }
}
