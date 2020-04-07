#ifndef MAPS_FREE_SPACE_MAP_H_
#define MAPS_FREE_SPACE_MAP_H_

#include <ros/ros.h>

#include <maps/single_map_layer.h>

#include <utils/json/json.h>
#include <utils/json/json_io.h>

#include "utils/map/free_space_structs.h"

namespace maps {

class FreeSpaceMapLayer : public SingleMapLayer<free_space::MapGraph>
{
 public:
  FreeSpaceMapLayer(const std::string& layer_name, const std::string& file_name);

 private:
  std::shared_ptr<free_space::MapGraph> loadFeatures(const std::string& file_name,
                                                     const MapFrame& target_frame);
};

}; // namespace maps

#endif // MAPS_FREE_SPACE_MAP_H_
