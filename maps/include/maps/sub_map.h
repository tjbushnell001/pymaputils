#ifndef MAPS_SUB_MAP_H_
#define MAPS_SUB_MAP_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "map_frame.h"

namespace maps {

template <typename MapTileType>
class SubMap
{
 public:
  /**
   * An sub-set of a tiled map.
   *
   * This is templatized on MapTileType, which represents the concrete tile
   * type.
   *
   * This may be sub-classed, particularly to add accessors.
   **/

  typedef MapTileType TileType;

  SubMap(const MapFrameType map_frame_type)
  {
    map_frame.type = map_frame_type;
  }

  /**
   * Get a tile by id.
   *
   * @param[in] tile_id The tile to get.
   * @returns The tile, or nullptr if not loaded.
   **/
  std::shared_ptr<MapTileType> getTile(uint64_t tile_id) const
  {
    auto it = tiles.find(tile_id);
    if (it == tiles.end()) {
      return nullptr;
    }
    return it->second;
  }

  /**
   * Get tile ids currently contained in this sub map.
   *
   * @returns Tile ids.
   **/
  std::unordered_set<uint64_t> getCurrentTileIds() const
  {
    std::unordered_set<uint64_t> current_tiles;
    for (const auto& tile : tiles) {
      current_tiles.insert(tile.first);
    }
    return current_tiles;
  }

  /**
   * Set the map frame, and clear current tiles.
   *
   * This doesn't convert the sub map to a new frame, it just clears the current
   * tiles.  While underwhelming, this is nonetheless useful for users which
   * are managing the frame conversion process.
   *
   * @param[in] map_frame The new map frame
   **/
  void resetFrame(const MapFrame& map_frame)
  {
    map_frame = map_frame;
    tiles.clear();
  }

  MapFrame map_frame;
  std::unordered_map<uint64_t, std::shared_ptr<MapTileType>> tiles;
  std::unordered_set<uint64_t> missing_tiles;
};

}; // namespace maps

#endif // MAPS_SUB_MAP_H_
