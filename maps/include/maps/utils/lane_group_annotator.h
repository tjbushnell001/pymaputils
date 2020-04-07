#ifndef LANE_MAP_SERVER_LANE_GROUP_ANNOTATOR_H
#define LANE_MAP_SERVER_LANE_GROUP_ANNOTATOR_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include "maps/map_layers.h"
#include "utils/map/lane_map_structs.h"

class LaneGroupAnnotator
{
 public:
  LaneGroupAnnotator(const std::string& route, const std::string& layer_name,
                     const maps::MapLayers& maps, bool lane_map::LaneGroup::*field);

  void annotateTile(lane_map::Tile& tile);

 private:
  const maps::MapLayers& maps_;
  const std::string route_name_;
  const std::string layer_name_;
  bool lane_map::LaneGroup::*lane_group_field_;
};

#endif // LANE_MAP_SERVER_MAP_ANNOTATOR_H
