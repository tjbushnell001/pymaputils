#ifndef MAPS_TILED_MAP_LAYER_H_
#define MAPS_TILED_MAP_LAYER_H_

#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <unordered_map>

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
   * @param[in] preload Preload tiles in the background.
   **/
  TiledMapLayer(MapLayerType layer_type, const std::string& layer_name, const std::string& dir_name,
                uint8_t tile_level, size_t tile_radius, bool preload)
    : MapLayer(layer_type, layer_name)
    , dir_name_(dir_name)
    , tile_level_(tile_level)
    , tile_radius_(tile_radius)
    , sub_map_(std::make_shared<SubMapType>())
    , preload_(preload)
  {
    if (preload_) {
      // start preload thread
      preload_thread_ = std::make_shared<std::thread>(&TiledMapLayer::runPreloadThread, this);
    }
  }

  ~TiledMapLayer()
  {
     // if we have a preload thread running, stop it
     if (preload_thread_) {
       running_ = false;
       preload_thread_->join();
       preload_thread_ = nullptr;
     }
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
    // sub map will be the same map frame type, but center and utm zone can change
    const auto new_map_frame = mapFrameFromLatLng(sub_map_->map_frame.type, lat, lng, utm_zone);

    // handle crossing UTM boundaries
    if (sub_map_->map_frame != new_map_frame) {
      // utm zone has changed, clear all tiles. They will be reloaded below
      ROS_WARN_STREAM("Changed Utm zone from "
                      << static_cast<int>(sub_map_->map_frame.utm_zone.zone) << " to "
                      << static_cast<int>(utm_zone.zone) << ", clearing tiles");

      if (preload_ || !sub_map_->transformFrame(new_map_frame)) {
        sub_map_->resetFrame(new_map_frame.type);
      }
    }

    sub_map_->map_frame = new_map_frame;

    // find the current tiles
    const auto current_tiles = map_utils::getSurroundingTiles(lat, lng, tile_radius_, tile_level_);

    if (!preload_) {
      // load new tiles
      loadTiles(sub_map_, current_tiles);
    } else {
      // only lock if we're doing preloading
      std::unique_lock<std::mutex> lock(preload_mutex_);

      preload_map_frame_ = new_map_frame;

      auto frame_it = preload_maps_.find(new_map_frame);
      if (frame_it != preload_maps_.end()) {
        const auto preload_map = frame_it->second;
        // copy any tiles we need from preloaded tiles
        for (const auto tile_id : current_tiles) {
          auto tile_it = preload_map->tiles.find(tile_id);
          if (tile_it != preload_map->tiles.end()) {
            sub_map_->tiles[tile_it->first] = tile_it->second;
          }
        }
      }
    }

    // drop outdated tiles
    unloadUnusedTiles(sub_map_, current_tiles);
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
   * Change the map frame type.
   *
   * This will clear the currently loaded features, and may require a call to
   * updateLocation() to reload them.
   *
   * @param[in] map_frame_type The new map frame type
   **/
  void resetFrame(const MapFrameType& map_frame_type)
  {
    ROS_WARN_STREAM("Changed map frame type from "
                    << mapFrameTypeStrings.at(sub_map_->map_frame.type)
                    << " to " << mapFrameTypeStrings.at(map_frame_type)
                    << ", clearing tiles");

    sub_map_->resetFrame(map_frame_type);
  }

  bool loadTiles(const std::unordered_set<uint64_t>& tile_ids)
  {
    return loadTiles(sub_map_, tile_ids);
  }

  bool unloadTile(const uint64_t tile_id)
  {
    return unloadTiles(sub_map_, tile_id);
  }

  void unloadUnusedTiles(const std::unordered_set<uint64_t>& keep_tile_ids)
  {
    unloadUnusedTiles(sub_map_, keep_tile_ids);
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
                                                                  const MapFrame& target_frame) const = 0;

 private:
  bool loadTiles(const std::shared_ptr<SubMapType>& sub_map,
                 const std::unordered_set<uint64_t>& tile_ids) const
  {
    bool loaded = false;
    for (const auto tile_id : tile_ids) {
      if (sub_map->tiles.find(tile_id) != sub_map->tiles.end()) {
        // already loaded
        continue;
      }

      if (sub_map->missing_tiles.find(tile_id) != sub_map->missing_tiles.end()) {
        // we already know tile doesn't exist
        continue;
      }

      // load the tile into the current map frame
      auto tile = loadTile(dir_name_, tile_id, sub_map->map_frame);
      if (tile) {
        // tile loaded
        sub_map->tiles[tile_id] = tile;
      } else {
        // tile was missing, keep track of it so we don't keep trying to load it
        sub_map->missing_tiles.insert(tile_id);
      }
      loaded = true;
    }
    return loaded;
  }

  bool unloadTile(const std::shared_ptr<SubMapType>& sub_map,
                  uint64_t tile_id) const
  {
    if (sub_map->tiles.find(tile_id) == sub_map->tiles.end()) {
      return false;
    }

    sub_map->tiles.erase(tile_id);
    return true;
  }

  void unloadUnusedTiles(const std::shared_ptr<SubMapType>& sub_map,
                         const std::unordered_set<uint64_t>& keep_tile_ids) const
  {
    // drop outdated tiles
    for (auto it = sub_map->tiles.begin(); it != sub_map->tiles.end();) {
      if (keep_tile_ids.count(it->first) == 0) {
        // tile no longer needed
        it = sub_map->tiles.erase(it);
      } else {
        // not erasing, move on to next tile
        ++it;
      }
    }
  }

  void runPreloadThread() {
    assert(preload_);

    ROS_INFO("Started map preloading thread.");

    while (running_ && ros::ok()) {
      std::unique_lock<std::mutex> lock(preload_mutex_);
      auto target_frame = preload_map_frame_;
      lock.unlock();

      if (target_frame.timestamp.isZero()) {
        // map frame never updated with location, don't know what to load
        ros::Duration(0.1).sleep();
        continue;
      }

      // Next, we load surrounding tiles into the preload map copy.
      // We use tile_radius_ + 1, so that we maintain a bigger radius than
      // we actually need, which is the main point of preloading.
      const double lat = target_frame.origin_latitude;
      const double lng = target_frame.origin_longitude;
      const auto current_tiles = map_utils::getSurroundingTiles(lat, lng, tile_radius_ + 1, tile_level_);

      // always add the target frame
      std::unordered_set<MapFrame> all_frames = {target_frame};
      if (target_frame.type == MapFrameType::UTM) {
        // if the target is UTM and we're near to a UTM zone boundary, we want
        // to load multiple frames so we're ready for the switch over

        // include the target UTM zone
        std::unordered_set<map_utils::UtmZone> all_zones = {target_frame.utm_zone};
        // as well as any others for other tiles we're about to load
        for (const auto tile_id : current_tiles) {
          const auto ll = map_utils::tileIdToLatLng(tile_id, tile_level_);
          const auto utm_zone = map_utils::getUtmZone(ll.first, ll.second);
          
          if (all_zones.count(utm_zone) == 0) {
            // this is a new UTM zone, add it as a new frame to preload
            MapFrame map_frame;
            map_frame.type  = MapFrameType::UTM;
            map_frame.utm_zone = utm_zone;

            all_frames.insert(map_frame);
            all_zones.insert(utm_zone);
          }
        }
      }

      for (const auto& map_frame : all_frames) {
        // grab the lock because we're going to update preload_map_
        lock.lock();

        std::shared_ptr<SubMapType> tmp_map;
        auto it = preload_maps_.find(map_frame);
        if (it != preload_maps_.end()) {
          // make a copy of preload_map_ so that we can do the loading without
          // holding the lock. The copy is fast because all the tiles are pointers
          tmp_map = std::make_shared<SubMapType>(*it->second);
        } else {
          ROS_WARN_STREAM("Adding preload frame "
                          << static_cast<int>(map_frame.type) << " zone "
                          << static_cast<int>(map_frame.utm_zone.zone));
          tmp_map = std::make_shared<SubMapType>(map_frame.type);
          tmp_map->map_frame = map_frame;
        }

        lock.unlock();

        // load new tiles
        loadTiles(tmp_map, current_tiles);

        // drop outdated tiles
        unloadUnusedTiles(tmp_map, current_tiles);

        // now that we've done the loading (which can be a little slow), grab
        // the lock again and (quickly) replace preload_map_ with tmp_map.
        lock.lock();

        // replace preloaded map
        preload_maps_[map_frame] = tmp_map;
        lock.unlock();
      }

      // remove any obscelete frames
      lock.lock();
      for (auto it = preload_maps_.begin(); it != preload_maps_.end();) {
        if (all_frames.count(it->first) == 0) {
          ROS_WARN_STREAM("Dropping preload frame "
                      << static_cast<int>(it->first.type) << " zone "
                      << static_cast<int>(it->first.utm_zone.zone));

          it = preload_maps_.erase(it);
        } else {
          ++it;
        }
      }
      lock.unlock();

      // sleep for half a second
      ros::Duration(0.5).sleep();
    }
  }

  std::string dir_name_;
  uint8_t tile_level_;
  size_t tile_radius_;
  std::shared_ptr<SubMapType> sub_map_;
  bool running_ = false;
  bool preload_ = false;
  std::unordered_map<MapFrame, std::shared_ptr<SubMapType>> preload_maps_;
  MapFrame preload_map_frame_;
  std::shared_ptr<std::thread> preload_thread_;
  std::mutex preload_mutex_;
};

}; // namespace maps

#endif // MAPS_TILED_MAP_LAYER_H_
