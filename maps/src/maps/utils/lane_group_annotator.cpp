#include "lane_group_annotator.h"
#include <utils/ros/params.h>
#include "map_parsing_utils.h"
#include "utils/file/file_utils.h"

/*
 * This class annotates Lane structs by setting boolean members.  Given a json file
 * containing polygons and a struct member, it will set the struct member to True if the
 * lane falls within a polygon.  Additionally, it filters
 * polygons so only those with a matching map file property are loaded.
 */
LaneGroupAnnotator::LaneGroupAnnotator(const std::string& route, const std::string& map_folder,
                                       const std::string& map_annotation_file,
                                       bool map_utils::LaneGroup::*field)
{
  lane_group_field_ = field;

  const std::string full_path = map_folder + "/annotations/" + map_annotation_file;
  if (!file_utils::fileExists(full_path)) {
    ROS_FATAL_STREAM("Lane Group annotation file " << full_path << " not found");
    std::terminate();
  }

  annotations_ = parseAnnotationFile(full_path, route);
  ROS_INFO_STREAM("Loaded " << annotations_.size() << " polygon(s) for route: " << route);
}


void LaneGroupAnnotator::annotateTile(map_utils::Tile& tile)
{
  /*
   *  Given a tile, annotate its lane group.  If any point in a lane exists within our polygons,
   *  set the lane group's member to true.  There's likely room for some optimization here
   *  as the list of polygons grows and it no longer makes sense to check every polygon.
   *  For now, this should be quick and low impact given this node only publishes once a second.
   */
  for (auto& lane_group_pair : tile.lane_groups) {
    auto& lane_group = lane_group_pair.second;
    for (const auto& annotation : annotations_) {
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
