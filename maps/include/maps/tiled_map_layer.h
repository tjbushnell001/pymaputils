#ifndef MAPS_TILED_MAP_LAYER_H_
#define MAPS_TILED_MAP_LAYER_H_

#include <ros/ros.h>
#include <memory>

#include <maps/map_layer.h>
#include <maps/sub_map.h>
#include "maps/map_frame.h"

#include "maps/utils/map_frame_utils.h"
#include "utils/file/file_utils.h"
#include "utils/map/utils.h"

namespace maps {

template <typename SubMapType>
class TiledMapLayer : public MapLayer
{
 public:
  /**
   * An tiled map layer, which keeps a fixed radius of tiles loaded in memory.
   *
   * Tiles are stored on disk by tile id, using morton encoded tile ids based
   * on lat, lng and a fixed tile level.
   *
   * This is templatized on SubMapType, must be specialization of SubMap, and
   * represents the collection of currently loaded tiles.  SubMap is, in turn,
   * templatized on MapTileType, which represents the concrete tile type for
   * each layer.
   *
   * This is an abstract class, which must be sub-classed.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   * @param[in] dir_name The directory where tiles are stored.
   * @param[in] tile_level The tile level used for this layer.
   * @param[in] tile_radius The tile radius at which we keep tiles loaded.
   **/
  TiledMapLayer(MapLayerType layer_type, const std::string& layer_name, const std::string& dir_name,
                uint8_t tile_level, size_t tile_radius)
    : MapLayer(layer_type, layer_name)
    , dir_name_(dir_name)
    , tile_level_(tile_level)
    , tile_radius_(tile_radius)
    , sub_map_(std::make_shared<SubMapType>())
  {
  }

  /**
   * This will be called once when a layer is loaded.
   **/
  void initialLoad()
  {
    // verify that tile directory exists
    if (!file_utils::fileExists(dir_name_)) {
      ROS_ERROR_STREAM(dir_name_ << ": Path not found");
    } else if (!file_utils::isDir(dir_name_)) {
      ROS_ERROR_STREAM(dir_name_ << ": Not a directory");
    }
  }

  /**
   * Update the map layer to be centered around the given coordinates.
   *
   * @param[in] lat latitude
   * @param[in] lng longitude
   * @param[in] utm_zone UTM zone to use for utm transforms.
   **/
  void updateLocation(const double lat, const double lng, const map_utils::UtmZone utm_zone)
  {
    // handle crossing UTM boundaries
    if (sub_map_->map_frame.type == MapFrameType::UTM && sub_map_->map_frame.utm_zone != utm_zone) {
      // utm zone has changed, clear all tiles. They will be reloaded below
      ROS_WARN_STREAM("Changed Utm zone from "
                      << static_cast<int>(sub_map_->map_frame.utm_zone.zone) << "to "
                      << static_cast<int>(utm_zone.zone) << ", clearing tiles");
      sub_map_->tiles.clear();
    }

    // sub map will be the same map frame type, but center and utm zone can change
    sub_map_->map_frame = mapFrameFromLatLng(sub_map_->map_frame.type, lat, lng, utm_zone);

    // find the current tiles
    const auto current_tiles = map_utils::getSurroundingTiles(lat, lng, tile_radius_, tile_level_);

    // add new tiles
    for (const auto tile_id : current_tiles) {
      if (sub_map_->tiles.find(tile_id) == sub_map_->tiles.end() &&
          sub_map_->missing_tiles.find(tile_id) == sub_map_->missing_tiles.end()) {
        // load the tile into the current map frame
        auto tile = loadTile(dir_name_, tile_id, sub_map_->map_frame);
        if (tile) {
          // tile loaded
          sub_map_->tiles[tile_id] = tile;
        } else {
          // tile was missing, keep track of it so we don't keep trying to load it
          sub_map_->missing_tiles.insert(tile_id);
        }
      }
    }
    // drop outdated tiles
    for (auto it = sub_map_->tiles.begin(); it != sub_map_->tiles.end();) {
      if (current_tiles.count(it->first) == 0) {
        // tile no longer needed
        it = sub_map_->tiles.erase(it);
      } else {
        // not erasing, move on to next tile
        ++it;
      }
    }
  }

  /**
   * Get the frame this layer is represented in.
   * @returns map frame
   **/
  MapFrame getMapFrame() const
  {
    return sub_map_->map_frame;
  }

  /**
   * Get map tiles for this layer.
   * @returns sub map
   **/
  std::shared_ptr<SubMapType> getSubMap() const
  {
    return sub_map_;
  }

  /**
   * Get the tile loading radius for this layer.
   *
   * @returns radius
   **/
  size_t getTileRadius() const
  {
    return tile_radius_;
  }

  /**
   * Set the tile loading radius for this layer.
   *
   * @param[in] tile_radius The radius of tiles.
   **/
  void setTileRadius(size_t tile_radius)
  {
    tile_radius_ = tile_radius;
  }

 protected:
  /**
   * Load the specified tile.
   *
   * This must be implemented by sub-classes.
   *
   * @param[in] dir_name Directory where tiles are stored.
   * @param[in] tile_id Id of tile to load.
   * @param[in] target_frame Map frame to convert tile into.
   **/
  virtual std::shared_ptr<typename SubMapType::TileType> loadTile(const std::string& dir_name,
                                                                  uint64_t tile_id,
                                                                  const MapFrame& target_frame) = 0;

 private:
  std::string dir_name_;
  uint8_t tile_level_;
  size_t tile_radius_;
  std::shared_ptr<SubMapType> sub_map_;
};

}; // namespace maps

#endif // MAPS_TILED_MAP_LAYER_H_
