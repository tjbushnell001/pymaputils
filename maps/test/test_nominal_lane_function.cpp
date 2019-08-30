#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>
#include <utils/ros/params.h>
#include "maps/utils/ego_lane_finder.h"
#include "maps/map_layers.h"
#include "maps/map_frame.h"
#include "maps/utils/map_frame_utils.h"
#include "utils/map/utils.h"
#include "diagnostics_utils/crash_handler.h"

auto JUNCTION_CONNECTOR_ID = 1;
auto JUNCTION_ID = 1;

auto IN_LANE_LG_ID = 1;
auto IN_LANE_ID = 1;

auto OUT_LANE_LG_ID = 100;
auto OUT_LANE_ID = 1;

const std::shared_ptr<maps::LaneSubMap> getLaneSubMapFromTile(uint64_t tile_id) {
  maps::MapLayers map(maps::createMapLayers({maps::MapLayerType::LANE}));

  // get lane map
  auto lane_map = map.getLayerAs<maps::LaneMapLayer>(maps::MapLayerType::LANE);

  EXPECT_NE(lane_map, nullptr);

  std::string map_dir;
  ros_params::getRequiredParam("/maps/map_dir", map_dir);

  auto utm_zone = map_utils::getUtmZone(0, 0);
  const auto map_frame = maps::mapFrameFromLatLng(maps::MapFrameType::GCS_NED, 0, 0, utm_zone);
  const auto tile_ptr = lane_map->loadTile(map_dir + "/tiles", tile_id, map_frame);

  EXPECT_NE(tile_ptr, nullptr);

  auto lane_sub_map = lane_map->getSubMap();
  lane_sub_map->tiles[tile_id] = tile_ptr;

  return lane_sub_map;
}

void testJunction(uint64_t tile_id) {
    const auto lane_sub_map = getLaneSubMapFromTile(tile_id);

    const auto junction_ref = lane_map::JunctionRef(tile_id, JUNCTION_CONNECTOR_ID, JUNCTION_ID);
    auto junction = lane_sub_map->getJunction(junction_ref);
    const auto in_nominal_lanes = ego_lane_finder::getNominalLanes(*lane_sub_map, junction->inflow_refs, lane_map_utils::TraverseDirection::IN);

    // Assert the size is 1 and the lane we get is the answer lane (always lg id 1, id 1)
    EXPECT_EQ(in_nominal_lanes.size(), 1);
    auto answer_lane_ref(lane_map::LaneRef(tile_id, IN_LANE_LG_ID, IN_LANE_ID));
    EXPECT_EQ(in_nominal_lanes[0], answer_lane_ref);

    const auto out_nominal_lanes = ego_lane_finder::getNominalLanes(*lane_sub_map, junction->outflow_refs, lane_map_utils::TraverseDirection::OUT);

    // Assert the size is 1 and the lane we get is the answer lane (always lg id 100, id 1)
    EXPECT_EQ(out_nominal_lanes.size(), 1);
    answer_lane_ref = lane_map::LaneRef(tile_id, OUT_LANE_LG_ID, OUT_LANE_ID);
    EXPECT_EQ(out_nominal_lanes[0], answer_lane_ref);
}

// Standard case, N/0 -> N/0
TEST(MapTileLoader, _N_0__N_0)
{
  const uint64_t TILE_ID = 1;
  testJunction(TILE_ID);
}

// Standard ramp exit, N/0 -> N/N
TEST(MapTileLoader, _N_0__N_N)
{
  const uint64_t TILE_ID = 2;
  testJunction(TILE_ID);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  crash_handler::install_crash_handler();

  // NOTE: createMapLayers() requires ros params.
  ros::init(argc, argv, "test_nominal_lane_function", ros::init_options::AnonymousName);
  ros::start();

  return RUN_ALL_TESTS();
}
