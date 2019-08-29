#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>
#include <utils/ros/params.h>

#include "maps/map_layers.h"
#include "maps/map_frame.h"
#include "maps/utils/map_frame_utils.h"
#include "utils/map/utils.h"

lane_map::Tile& getTile(uint64_t tile_id) {
  maps::MapLayers map(maps::createMapLayers({maps::MapLayerType::LANE}));

  // get lane map
  auto lane_map = map.getLayerAs<maps::LaneMapLayer>(maps::MapLayerType::LANE);

  EXPECT_NE(lane_map, nullptr);

  std::string map_dir;
  ros_params::getRequiredParam("/maps/map_dir", map_dir);

  auto utm_zone = map_utils::getUtmZone(0, 0);
  const auto map_frame = maps::mapFrameFromLatLng(maps::MapFrameType::GCS_NED, 0, 0, utm_zone);
  const auto tile_ptr = lane_map->loadTile(map_dir + "/tiles", tile_id, map_frame);

  //  auto lane_submap = lane_map->getSubMap();
  EXPECT_NE(tile_ptr, nullptr);

  return *tile_ptr;
}


TEST(MapTileLoader, N_0)
{

  const auto& tile = getTile(1);

  for (auto& lg_pair : tile.lane_groups) {
    std::cout << std::endl;
    std::cout << "HERE" << std::endl;
    std::cout << lg_pair.first <<std::endl;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // NOTE: createMapLayers() requires ros params.
  ros::init(argc, argv, "test_nominal_lane_function", ros::init_options::AnonymousName);
  ros::start();

  return RUN_ALL_TESTS();
}
