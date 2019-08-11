#include "lane_group_annotator.h"

/*
 * This class annotates Lane structs by setting boolean members.  Given a json file
 * containing polygons and a struct member, it will set the struct member to True if the
 * lane falls within a polygon.  Additionally, it filters
 * polygons so only those with a matching map file property are loaded.
 */
LaneGroupAnnotator::LaneGroupAnnotator(const std::string& route, const std::string& layer_name,
                                       const maps::MapLayers& maps,
                                       bool lane_map::LaneGroup::*field)
  : maps_(maps), route_name_(route), layer_name_(layer_name), lane_group_field_(field)
{
}

void LaneGroupAnnotator::annotateTile(lane_map::Tile& tile)
{
  /*
   *  Given a tile, annotate its lane group.  If any point in a lane exists within our polygons,
   *  set the lane group's member to true.  There's likely room for some optimization here
   *  as the list of polygons grows and it no longer makes sense to check every polygon.
   *  For now, this should be quick and low impact given this node only publishes once a second.
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

  for (auto& lane_group_pair : tile.lane_groups) {
    auto& lane_group = lane_group_pair.second;
    for (const auto& annotation : route_polygons) {
      // early if already set to true::
      if (lane_group.*lane_group_field_) {
        continue;
      }

      // check left boundary
      for (const auto& pt : lane_group.left_boundary) {
        const BoostPoint b_pt(pt.x, pt.y);
        if (boost::geometry::within(b_pt, annotation)) {
          lane_group.*lane_group_field_ = true;
          break;
        }
      }

      // early  exitif already set to true::
      if (lane_group.*lane_group_field_) {
        continue;
      }

      // check right boundary
      for (const auto& pt : lane_group.right_boundary) {
        const BoostPoint b_pt(pt.x, pt.y);
        if (boost::geometry::within(b_pt, annotation)) {
          lane_group.*lane_group_field_ = true;
          break;
        }
      }
    }
  }
}
