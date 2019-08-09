#ifndef MAPS_MAP_LAYERS_H_
#define MAPS_MAP_LAYERS_H_

#include <maps/map_layer.h>
#include <boost/functional/hash.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

#include <maps/free_space_maps.h>
#include <maps/geojson_maps.h>

namespace maps {

class MapLayers
{
 public:
  /**
   * A container of map layers.
   *
   * 1. Each layer is identified by a layer type and a layer name.
   *   Layers with multiple files of the same layer type (e.g. free space),
   *   will be_named_ and the layer name will be meaningful.
   *
   *   Layers which only have one set of files (e.g. lanes), will be _unnamed_
   *   and the layer name will be the empty string.  Several methods have an
   *   optional layer_name for this purpose.
   *
   * 2. Use createLayers() to create actual layers.  For example:
   *   maps::MapLayers(maps::createMapLayers({maps::MapLayerType::LANE_MAPS,
   *                                          maps::MapLayerType::FREE_SPACE,
   *                                         }));
   *
   * 3. Use getLayerAs() and getAllNamedLayersAs() to get layers. For example:
   *   map.getLayerAs<maps::LaneMapLayer>(MapLayerType::LANE_MAPS);
   *   map.getAllTypeLayersAs<maps::FreeSpaceMapLayer>(MapLayerType::FREE_SPACE);
   **/

  MapLayers();

  MapLayers(const std::vector<std::shared_ptr<MapLayer>>& layers);

  /**
   * Add a layer to the collection.
   *
   * @param[in] layer The layer to add.
   **/
  void addLayer(std::shared_ptr<MapLayer> layer);

  /**
   * Add multiple layers to the collection.
   *
   * @param[in] layers The layers to add.
   **/
  void addLayers(const std::vector<std::shared_ptr<MapLayer>>& layers);

  /**
   * Check if a layer exists in the collection.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  This is optional, which is
   *                       useful for unnamed layers.
   * @returns boolean indicating existance.
   **/
  bool hasLayer(MapLayerType layer_type, const std::string& layer_name = "") const;

  /**
   * Get a layer from the collection, if it exists.
   *
   * NOTE: This returns the abstract MapLayer, which has limited functionality.
   *       Most users will want getLayerAs() and getAllNamedLayersAs().
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  This is optional, which is
   *                       useful for unnamed layers.
   * @returns Abstract MapLayer, or nullptr if not found.
   **/
  std::shared_ptr<MapLayer> getLayer(MapLayerType layer_type,
                                     const std::string& layer_name = "") const;

  /**
   * Get a layer from the collection, if it exists.
   *
   * This is templatized on MapLayer, which is expected to be the concrete type
   * of the map layer.  This will upcast the layer appropriately (or to nullptr
   * if the type is wrong).
   * The pattern for using this is:
   *   map.getLayerAs<maps::GeoJsonMapLayer>(MapLayerType::FREE_SPACE, "foo");
   *
   * This method is useful for unnamed layers and specific named layers. To
   * get all of the named layers for a layer type, see getAllTypeLayersAs().
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  This is optional, which is
   *                       useful for unnamed layers.
   * @returns The map layer, or nullptr if not found or type is wrong.
   **/
  template <typename ML>
  std::shared_ptr<ML> getLayerAs(MapLayerType layer_type, const std::string& layer_name = "") const
  {
    auto layer = getLayer(layer_type, layer_name);
    return std::dynamic_pointer_cast<ML>(layer);
  }

  /**
   * Get all layers with the given layer type from the collection.
   *
   * This is templatized on MapLayer, see getLayerAs() for notes.
   *
   * This method is particularly useful for named layers.
   *
   * @param[in] layer_type The type of the layer.
   * @returns A vector of maps layers.
   **/
  template <typename ML>
  std::vector<std::shared_ptr<ML>> getAllTypeLayersAs(MapLayerType layer_type) const
  {
    std::vector<std::shared_ptr<ML>> results;
    for (const auto& layer_pair : layers_) {
      if (layer_pair.first.first != layer_type) {
        continue;
      }
      auto layer = std::dynamic_pointer_cast<ML>(layer_pair.second);
      if (layer) {
        results.push_back(layer);
      }
    }
    return results;
  }

  /**
   * Get all layers types and names in the collection.
   *
   * @returns A vector of layer type/name pairs.
   **/
  std::vector<std::pair<MapLayerType, std::string>> getLayerNames() const;

  /**
   * Update all map layers to be centered around the given coordinates.
   *
   * @param[in] lat latitude
   * @param[in] lng longitude
   * @param[in] utm_zone UTM zone to use for utm transforms.
   **/
  void updateLocation(const double lat, const double lng, const map_utils::UtmZone utm_zone);

 private:
  std::unordered_map<std::pair<MapLayerType, std::string>, std::shared_ptr<MapLayer>,
                     boost::hash<std::pair<MapLayerType, std::string>>>
      layers_;
};

/**
 * Factory function to load all layers for the requested layer types.
 *
 * @param[in] layer_types Layer types to load.
 **/
std::vector<std::shared_ptr<MapLayer>>
createMapLayers(const std::vector<MapLayerType>& layer_types);

}; // namespace maps

#endif // MAPS_MAP_LAYERS
