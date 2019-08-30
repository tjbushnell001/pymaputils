#ifndef MAPS_ROAD_MAP_H_
#define MAPS_ROAD_MAP_H_

#include <ros/ros.h>
#include <memory>

#include <maps/tiled_map_layer.h>

#include <utils/json/json.h>
#include <utils/json/json_io.h>

#include "utils/map/road_graph_structs.h"

namespace maps {

class RoadSubMap : public SubMap<road_map::RoadTile>
{
 public:
  RoadSubMap(const MapFrameType frame_type = MapFrameType::GCS);

  const road_map::RoadSegment* getRoadSegment(const road_map::RoadSegmentRef& ref) const;
  const road_map::RoadConnector* getRoadConnector(const road_map::RoadConnectorRef& ref) const;
};

class RoadMapLayer : public TiledMapLayer<RoadSubMap>
{
 public:
  RoadMapLayer(const std::string& dir_name, size_t tile_radius, bool preload = true);

  std::shared_ptr<road_map::RoadTile> loadTile(const std::string& dir_name, uint64_t tile_id,
                                               const MapFrame& target_frame) const;
};

}; // namespace maps

#endif // MAPS_ROAD_MAP_H_
