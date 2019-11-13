#include <maps/geojson_maps.h>

using namespace maps;

GeoJsonSubMap::GeoJsonSubMap(const MapFrameType frame_type) : SubMap(frame_type)
{
}

GeoJsonTileMapLayer::GeoJsonTileMapLayer(MapLayerType layer_type, const std::string& layer_name,
                                         const std::string& dir_name, uint8_t tile_level,
                                         size_t tile_radius, bool preload)
  : TiledMapLayer(layer_type, layer_name, dir_name, tile_level, tile_radius, preload)
{
}

std::shared_ptr<Json::Value> GeoJsonTileMapLayer::loadTile(const std::string& dir_name,
                                                           uint64_t tile_id,
                                                           const MapFrame& target_frame) const
{
  assert(target_frame.type == MapFrameType::GCS);

  const std::string fn = dir_name + "/" + std::to_string(tile_id) + ".json";
  if (!file_utils::fileExists(fn)) {
    return nullptr;
  }

  auto root = std::make_shared<Json::Value>();
  if (!utils_json::readJsonFile(fn, root.get())) {
    return nullptr;
  }
  return root;
}

GeoJsonMapLayer::GeoJsonMapLayer(MapLayerType layer_type, const std::string& layer_name,
                                 const std::string& file_name)
  : SingleMapLayer(layer_type, layer_name, MapFrameType::GCS, file_name)
{
}

std::shared_ptr<Json::Value> GeoJsonMapLayer::loadFeatures(const std::string& filename,
                                                           const MapFrame& target_frame)
{
  assert(target_frame.type == MapFrameType::GCS);

  auto root = std::make_shared<Json::Value>();
  if (!utils_json::readJsonFile(filename, root.get())) {
    return nullptr;
  }
  return root;
}
