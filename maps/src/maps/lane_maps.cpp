#include <maps/lane_maps.h>
#include "maps/utils/lane_map_transform.h"
#include "utils/map/lane_map_parser.h"

using namespace maps;

LaneMapLayer::LaneMapLayer(const std::string& dir_name, size_t tile_radius)
  : TiledMapLayer(MapLayerType::LANE, "", dir_name, 14, tile_radius)
{
}

std::shared_ptr<lane_map::Tile> LaneMapLayer::loadTile(const std::string& dir_name,
                                                       uint64_t tile_id,
                                                       const MapFrame& target_frame)
{
  assert(target_frame.type == MapFrameType::GCS_NED || target_frame.type == MapFrameType::UTM);

  const std::string fn = dir_name + "/" + std::to_string(tile_id) + ".json";
  if (!file_utils::fileExists(fn)) {
    return nullptr;
  }

  Json::Value root;
  if (!utils_json::readJsonFile(fn, &root)) {
    return nullptr;
  }

  auto tile = std::make_shared<lane_map::Tile>(lane_map::parseTile(root));
  tile->id = tile_id;

  if (target_frame.type == MapFrameType::UTM) {
    // convert tile to UTM
    const bool use_NED = target_frame.type == maps::MapFrameType::GCS_NED;
    maps::transformTileGpsToUtm(tile.get(), target_frame.utm_zone, use_NED);
  }

  return tile;
}

LaneSubMap::LaneSubMap(const MapFrameType frame_type) : SubMap(frame_type)
{
}

const lane_map::LaneGroup* LaneSubMap::getLaneGroup(const lane_map::LaneGroupRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto lane_group_it = tile->lane_groups.find(ref);
  if (lane_group_it == tile->lane_groups.end()) {
    return nullptr;
  }
  return &lane_group_it->second;
}

const lane_map::Lane* LaneSubMap::getLane(const lane_map::LaneRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto lane_group_it = tile->lane_groups.find(ref.getLaneGroupRef());
  if (lane_group_it == tile->lane_groups.end()) {
    return nullptr;
  }
  const auto& lane_group = lane_group_it->second;

  const auto lane_it = lane_group.lanes.find(ref);
  if (lane_it == lane_group.lanes.end()) {
    return nullptr;
  }

  return &lane_it->second;
}

const lane_map::Connector* LaneSubMap::getConnector(const lane_map::ConnectorRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto connector_it = tile->connectors.find(ref);
  if (connector_it == tile->connectors.end()) {
    return nullptr;
  }
  return &connector_it->second;
}

const lane_map::Boundary* LaneSubMap::getBoundary(const lane_map::BoundaryRef& ref) const
{
  const auto tile = getTile(ref.tile_id);
  if (!tile) {
    return nullptr;
  }
  const auto lane_group_it = tile->lane_groups.find(ref.getLaneGroupRef());
  if (lane_group_it == tile->lane_groups.end()) {
    return nullptr;
  }
  const auto& lane_group = lane_group_it->second;

  const auto boundary_it = lane_group.boundaries.find(ref);
  if (boundary_it == lane_group.boundaries.end()) {
    return nullptr;
  }

  return &boundary_it->second;
}

void LaneSubMap::addLaneGroup(const lane_map::LaneGroupRef& ref,
                              const lane_map::LaneGroup& lane_group)
{
  auto tile = getTile(ref.tile_id);
  if (!tile) {
    // create a new tile
    tile = std::make_shared<lane_map::Tile>();
    // set tile id
    tile->id = ref.tile_id;

    tiles[ref.tile_id] = tile;
  }
  tile->lane_groups[ref] = lane_group;
}

void LaneSubMap::addConnector(const lane_map::ConnectorRef& ref,
                              const lane_map::Connector& connector)
{
  auto tile = getTile(ref.tile_id);
  if (!tile) {
    // create a new tile
    tile = std::make_shared<lane_map::Tile>();
    // set tile id
    tile->id = ref.tile_id;

    tiles[ref.tile_id] = tile;
  }
  tile->connectors[ref] = connector;
}

PolygonFeatureMapLayer::PolygonFeatureMapLayer(MapLayerType layer_type,
                                               const std::string& layer_name,
                                               MapFrameType frame_type,
                                               const std::string& file_name)
  : SingleMapLayer(layer_type, layer_name, frame_type, file_name)
{
}

std::shared_ptr<std::vector<lane_map::PolygonFeature>>
PolygonFeatureMapLayer::loadFeatures(const std::string& filename, const MapFrame& target_frame)
{
  assert(target_frame.type == MapFrameType::GCS || target_frame.type == MapFrameType::GCS_NED);

  const bool use_NED = target_frame.type == MapFrameType::GCS_NED;

  Json::Value root;
  if (!utils_json::readJsonFile(filename, &root)) {
    return nullptr;
  }

  auto map = std::make_shared<std::vector<lane_map::PolygonFeature>>(
      lane_map::parsePolygonFeature(root, use_NED));
  return map;
}
