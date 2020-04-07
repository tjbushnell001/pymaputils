#include <diagnostics_utils/instrumentation.h>
#include <maps/road_maps.h>
#include "utils/map/road_graph_parser.h"

using namespace maps;
using namespace diagnostics_utils;


RoadSubMap::RoadSubMap(const MapFrameType frame_type) : SubMap(frame_type)
{
}

RoadMapLayer::RoadMapLayer(const std::string& dir_name, size_t tile_radius, bool preload)
  : TiledMapLayer(MapLayerType::ROAD, "", dir_name, 10, tile_radius, preload)
{
}

std::shared_ptr<road_map::RoadTile> RoadMapLayer::loadTile(const std::string& dir_name,
                                                           uint64_t tile_id,
                                                           const MapFrame& target_frame) const
{
  assert(target_frame.type == MapFrameType::GCS);

  const std::string fn = dir_name + "/" + std::to_string(tile_id) + ".json";
  if (!file_utils::fileExists(fn)) {
    return nullptr;
  }

  SequentialExecution exec_timing;
  exec_timing.trace();

  exec_timing.start("readJsonFile", CALLER_INFO());
  Json::Value root;
  if (!utils_json::readJsonFile(fn, &root)) {
    return nullptr;
  }

  return std::make_shared<road_map::RoadTile>(road_map::parseRoadTile(root));
}

const road_map::RoadSegment* RoadSubMap::getRoadSegment(const road_map::RoadSegmentRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto road_segment_it = tile->road_segments.find(ref);
  if (road_segment_it == tile->road_segments.end()) {
    return nullptr;
  }
  return &road_segment_it->second;
}

const road_map::RoadConnector*
RoadSubMap::getRoadConnector(const road_map::RoadConnectorRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto road_connector_it = tile->road_connectors.find(ref);
  if (road_connector_it == tile->road_connectors.end()) {
    return nullptr;
  }
  return &road_connector_it->second;
}
