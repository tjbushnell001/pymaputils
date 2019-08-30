#ifndef MAPS_GEO_JSON_MAP_H_
#define MAPS_GEO_JSON_MAP_H_

#include <ros/ros.h>

#include <maps/single_map_layer.h>
#include <maps/tiled_map_layer.h>

#include <utils/json/json.h>
#include <utils/json/json_io.h>

namespace maps {

class GeoJsonSubMap : public SubMap<Json::Value>
{
 public:
  GeoJsonSubMap(const MapFrameType frame_type = MapFrameType::GCS);
};

class GeoJsonTileMapLayer : public TiledMapLayer<GeoJsonSubMap>
{
 public:
  /**
   * A tiled GeoJson map layer.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   * @param[in] dir_name The directory where tiles are stored.
   * @param[in] tile_level The tile level used for this layer.
   * @param[in] tile_radius The tile radius at which we keep tiles loaded.
   **/
  GeoJsonTileMapLayer(MapLayerType layer_type, const std::string& layer_name,
                      const std::string& dir_name, uint8_t tile_level, size_t tile_radius, bool preload = true);

  std::shared_ptr<Json::Value> loadTile(const std::string& dir_name, uint64_t tile_id,
                                        const MapFrame& target_frame) const;
};

class GeoJsonMapLayer : public SingleMapLayer<Json::Value>
{
 public:
  /**
   * A single file GeoJson map layer.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   * @param[in] file_name The filename of the map file.
   **/
  GeoJsonMapLayer(MapLayerType layer_type, const std::string& layer_name,
                  const std::string& file_name);

 private:
  std::shared_ptr<Json::Value> loadFeatures(const std::string& file_name,
                                            const MapFrame& target_frame);
};

}; // namespace maps

#endif // MAPS_GEO_JSON_MAP_H_
