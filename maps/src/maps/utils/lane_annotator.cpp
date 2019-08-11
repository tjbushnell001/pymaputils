#include "lane_annotator.h"

/*
 * This class annotates Lane structs by setting boolean members.  Given a json file
 * containing polygons and a struct member, it will set the struct member to True if the
 * lane falls within a polygon.  Additionally, it filters
 * polygons so only those with a matching map file property are loaded.
 */
LaneAnnotator::LaneAnnotator(const std::string& route, const std::string& layer_name,
                             const maps::MapLayers& maps, bool lane_map::Lane::*field)
  : maps_(maps), route_name_(route), layer_name_(layer_name), lane_field_(field)
{
}


void LaneAnnotator::annotateTile(lane_map::Tile& tile)
{
  /*
   *  Given a tile, annotate its lanes.  If any point in a lane exists within our polygons, set the
   * lane's member
   *  to true.  There's likely room for some optimization here as the list of polygons grows and it
   * no longer makes
   *  sense to check every polygon.  For now, this should be quick and low impact given this node
   * only publishes
   *  once a second.
   */
  auto annotation_map = maps_.getLayerAs<maps::PolygonFeatureMapLayer>(
      maps::MapLayerType::LANE_ANNOTATION, layer_name_);
  // Make sure map layer was loaded
  assert(annotation_map);

  std::vector<BoostPolygon> route_polygons;
  for (const auto& route : *annotation_map->getFeatures()) {
    const Json::Value json_route = route.properties["route"];
    if (json_route.isNull() || json_route.asString() != route_name_) {
      continue;
    }
    route_polygons.push_back(route.polygon);
  }

  if (route_polygons.empty()) {
    // no annotations for route
    return;
  }

  for (auto& lane_group : tile.lane_groups) {
    for (auto& lane : lane_group.second.lanes) {
      // Slight optimization: if this field is already set to true, skip to the next lane
      if (lane.second.*lane_field_)
        continue;

      for (auto& lane_msg_pt : lane.second.pts) {
        // Another small optimization: break out of this loop if we've already flipped the bool
        if (lane.second.*lane_field_)
          break;

        const BoostPoint lane_pt(lane_msg_pt.x, lane_msg_pt.y);
        for (const auto& annotation : route_polygons) {
          if (boost::geometry::within(lane_pt, annotation)) {
            lane.second.*lane_field_ = true;
            break;
          }
        }
      }
    }
  }
}
