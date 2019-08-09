#include "lane_annotator.h"
#include "map_parsing_utils.h"
#include "utils/file/file_utils.h"

/*
 * This class annotates Lane structs by setting boolean members.  Given a json file
 * containing polygons and a struct member, it will set the struct member to True if the
 * lane falls within a polygon.  Additionally, it filters
 * polygons so only those with a matching map file property are loaded.
 */
LaneAnnotator::LaneAnnotator(const std::string& route, const std::string& map_folder,
                             const std::string& map_annotation_file, bool map_utils::Lane::*field)
{
  lane_field_ = field;

  const std::string full_path = map_folder + "/annotations/" + map_annotation_file;

  if (!file_utils::fileExists(full_path)) {
    ROS_FATAL_STREAM("Lane annotation file " << full_path << " not found");
    std::terminate();
  }

  annotations_ = parseAnnotationFile(full_path, route);
  ROS_INFO_STREAM("Loaded " << annotations_.size() << " polygon(s) for route: " << route);
}


void LaneAnnotator::annotateTile(map_utils::Tile& tile)
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
        for (const auto& annotation : annotations_) {
          if (boost::geometry::within(lane_pt, annotation)) {
            lane.second.*lane_field_ = true;
            break;
          }
        }
      }
    }
  }
}
