#ifndef MAPS_MAP_LAYER_H_
#define MAPS_MAP_LAYER_H_

#include <ros/ros.h>
#include "maps/map_frame.h"
#include "utils/map/utils.h"

namespace maps {

enum class MapLayerType
{
  LANE,
  LANE_ANNOTATION,
  ROAD,
  MAP_READER,
  FREE_SPACE,
  RADAR_ZONES,
  LOCALIZATION_ZONES
};

class MapLayer
{
 public:
  /**
   * An individual map layer.
   *
   * Each layer is identified by a layer type, as well a layer name for cases
   * where there are multiple files of the same type (e.g. free space).
   *
   * Layers are capable of auto-transforming into alternative frames (e.g. UTM).
   * They maintain their current MapFrame for this purpose, although supported
   * frames vary by sub-class implementation.
   *
   * This is an abstract class, which must be sub-classed.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   **/
  MapLayer(MapLayerType layer_type, const std::string& layer_name);

  /**
   * Update the map layer to be centered around the given coordinates.
   *
   * Tiled sub-classes should use this to manage their loaded tiles.  Other
   * sub-classes may choose to use this only for utm zone updates.
   *
   * @param[in] lat latitude
   * @param[in] lng longitude
   * @param[in] utm_zone UTM zone to use for utm transforms.
   **/
  virtual void updateLocation(const double lat, const double lng,
                              const map_utils::UtmZone utm_zone) = 0;

  /**
   * This will be called once when a layer is loaded.
   *
   * Single file sub-classes should use this to load features.  Tiled
   * sub-classes may choose to ignore this, or to do other setup work.
   **/
  virtual void initialLoad() = 0;

  /**
   * Get the layer type.
   * @returns layer type
   **/
  MapLayerType getLayerType() const;

  /**
   * Get the layer name.  May be an empty string.
   * @returns layer name
   **/
  const std::string& getLayerName() const;

  /**
   * Get the frame this layer is represented in.
   * @returns map frame
   **/
  virtual MapFrame getMapFrame() const = 0;

 private:
  MapLayerType layer_type_;
  std::string layer_name_;
};

}; // namespace maps

#endif // MAPS_MAP_LAYER_H_
