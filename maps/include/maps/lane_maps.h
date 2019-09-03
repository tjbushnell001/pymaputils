#ifndef MAPS_LANE_MAP_H_
#define MAPS_LANE_MAP_H_

#include <ros/ros.h>

#include <maps/single_map_layer.h>
#include <maps/tiled_map_layer.h>

#include <utils/json/json.h>
#include <utils/json/json_io.h>

#include "utils/map/lane_map_structs.h"

namespace maps {

class LaneSubMap : public SubMap<lane_map::Tile>
{
 public:
 /**
   * A tile lane sub map, which exposes accessors and settors.
   *
   **/
  LaneSubMap(const MapFrameType frame_type = MapFrameType::GCS_NED);

  /**
   * Transforms the current tiles into a new map frame.
   *
   * This leaves it to sub-classes to decide what sort of conversions they support.
   * The default implementation coverts nothing.
   *
   * @param[in] target_frame The new map frame
   * @returns bool indicating whether conversion was successful.
   **/
  virtual bool transformFrame(const MapFrame& target_frame);

  const lane_map::LaneGroup* getLaneGroup(const lane_map::LaneGroupRef& ref) const;
  const lane_map::Lane* getLane(const lane_map::LaneRef& ref) const;
  const lane_map::Connector* getConnector(const lane_map::ConnectorRef& ref) const;
  const lane_map::Boundary* getBoundary(const lane_map::BoundaryRef& ref) const;

  void addLaneGroup(const lane_map::LaneGroupRef& ref, const lane_map::LaneGroup& lane_group);
  void addConnector(const lane_map::ConnectorRef& ref, const lane_map::Connector& connector);
};

class LaneMapLayer : public TiledMapLayer<LaneSubMap>
{
 public:
 /**
   * A tiled lane map layer.
   *
   * This loads tiled lane maps from our custom json format.
   *
   * @param[in] dir_name The directory where tiles are stored.
   * @param[in] tile_radius The tile radius at which we keep tiles loaded.
   * @param[in] preload Asyncronously and preemptively load tiles.
   **/
  LaneMapLayer(const std::string& dir_name, size_t tile_radius, bool preload = true);

 private:
  std::shared_ptr<lane_map::Tile> loadTile(const std::string& dir_name, uint64_t tile_id,
                                           const MapFrame& target_frame) const;
};

class PolygonFeatureMapLayer : public SingleMapLayer<std::vector<lane_map::PolygonFeature>>
{
 public:
 /**
   * A polygon feature map layer.
   *
   * This will load a GeoJson file containing Polygon features, along with any
   * associated properties.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   * @param[in] frame_type Desired frame representation (either GCS or GCS_NED,
   *                       to swap x,y points).
   * @param[in] file_name The filename of the map file.
   **/
  PolygonFeatureMapLayer(MapLayerType layer_type, const std::string& layer_name,
                         MapFrameType frame_type, const std::string& file_name);

 private:
  std::shared_ptr<std::vector<lane_map::PolygonFeature>> loadFeatures(const std::string& file_name,
                                                                      const MapFrame& target_frame);
};

}; // namespace maps

#endif // MAPS_LANE_MAP_H_
