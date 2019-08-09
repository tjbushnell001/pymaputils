#include <maps/free_space_maps.h>
#include "utils/map/free_space_parser.h"

using namespace maps;

FreeSpaceMapLayer::FreeSpaceMapLayer(const std::string& layer_name, const std::string& file_name)
  : SingleMapLayer(MapLayerType::FREE_SPACE, layer_name, MapFrameType::GCS_NED, file_name)
{
}

std::shared_ptr<free_space::MapGraph> FreeSpaceMapLayer::loadFeatures(const std::string& filename,
                                                                      const MapFrame& target_frame)
{
  assert(target_frame.type == MapFrameType::GCS_NED);

  Json::Value root;
  if (!utils_json::readJsonFile(filename, &root)) {
    return nullptr;
  }

  auto map = std::make_shared<free_space::MapGraph>();
  free_space::map_parser::parseMap(root, map.get());

  // set the layer name inside the struct
  map->zone.map_name = getLayerName();

  return map;
}
