#ifndef LANE_MAP_SERVER_LANE_ANNOTATOR_H
#define LANE_MAP_SERVER_LANE_ANNOTATOR_H

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include "utils/json/json.h"
#include "utils/map/structs.h"

class LaneAnnotator
{
 public:
  LaneAnnotator(const std::string& route, const std::string& map_folder,
                const std::string& map_annotation_file, bool map_utils::Lane::*field);

  void annotateTile(map_utils::Tile& tile);

 private:
  std::vector<BoostPolygon> annotations_;
  bool map_utils::Lane::*lane_field_;
};

#endif // LANE_MAP_SERVER_MAP_ANNOTATOR_H
