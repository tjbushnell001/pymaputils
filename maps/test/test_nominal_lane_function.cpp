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

// Standard ramp entrance to normal lane, 0/N->N/0
TEST(MapTileLoader, _0_N__N_0)
{
  const uint64_t TILE_ID = 3;
  testJunction(TILE_ID);
}

// N/0->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 4;
  testJunction(TILE_ID);
}

// N/N->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 5;
  testJunction(TILE_ID);
}

// NM/0->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 6;
  testJunction(TILE_ID);
}

// 0/N->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 7;
  testJunction(TILE_ID);
}

// N/0->M/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 8;
  testJunction(TILE_ID);
}

// 0/N->M/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 9;
  testJunction(TILE_ID);
}

// S/0->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 10;
  testJunction(TILE_ID);
}

// N/0->NS/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 11;
  testJunction(TILE_ID);
}

// N/0->2N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 12;
  testJunction(TILE_ID);
}

// 2N/0->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 13;
  testJunction(TILE_ID);
}

// N/0->0/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 14;
//  testJunction(TILE_ID);
//}

// N/0->0/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 15;
  testJunction(TILE_ID);
}

// N/0->0/NS
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 16;
  testJunction(TILE_ID);
}

// 0/N->N/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 17;
  testJunction(TILE_ID);
}

// 0/NM->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 18;
  testJunction(TILE_ID);
}

// 0/NM->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 19;
  testJunction(TILE_ID);
}

// 0/0->N/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 20;
//  testJunction(TILE_ID);
//}

// 0/N->0/2N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 21;
  testJunction(TILE_ID);
}

// 0/S->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 22;
  testJunction(TILE_ID);
}

// 0/N->0/M
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 23;
  testJunction(TILE_ID);
}

// 0/2N->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 24;
  testJunction(TILE_ID);
}

// NM/0->N/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 25;
  testJunction(TILE_ID);
}

// 0/N->0/NS
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 26;
  testJunction(TILE_ID);
}

// N/0->N2S/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 27;
  testJunction(TILE_ID);
}

// NM/0->NS/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 28;
  testJunction(TILE_ID);
}

// N/0->N/NS
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 29;
  testJunction(TILE_ID);
}

// N/NM->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 30;
  testJunction(TILE_ID);
}

// 0/0->0/N
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 31;
//  testJunction(TILE_ID);
//}

// 0/N->0/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 32;
//  testJunction(TILE_ID);
//}

// N2M/0->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 33;
  testJunction(TILE_ID);
}

// 0/2N->0/M
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 34;
  testJunction(TILE_ID);
}

// N/N->S/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 35;
  testJunction(TILE_ID);
}

// S/0->M/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 36;
  testJunction(TILE_ID);
}

// 0/N->0/S
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 37;
  testJunction(TILE_ID);
}

// M/0->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 38;
  testJunction(TILE_ID);
}

// N/0->2NS/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 39;
  testJunction(TILE_ID);
}

// 2N/0->N/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 40;
//  testJunction(TILE_ID);
//}

// NM/0->0/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 41;
  testJunction(TILE_ID);
}

// N/N->M/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 42;
  testJunction(TILE_ID);
}

// N/0->NS/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 43;
  testJunction(TILE_ID);
}

// 0/N2M->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 44;
  testJunction(TILE_ID);
}

// NM/0->N/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 45;
//  testJunction(TILE_ID);
//}

// 0/0->0/M
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 46;
//  testJunction(TILE_ID);
//}

// 0/S->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 47;
  testJunction(TILE_ID);
}

// NM/N->N/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 48;
  testJunction(TILE_ID);
}

// N/0->N/S
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 49;
  testJunction(TILE_ID);
}

// NM/N->NS/0
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 50;
  testJunction(TILE_ID);
}

// 0/0->M/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 51;
//  testJunction(TILE_ID);
//}

// N/0->2NS/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 52;
  testJunction(TILE_ID);
}

// N/N->N/N
TEST(MapTileLoader, _)
{
  const uint64_t TILE_ID = 53;
  testJunction(TILE_ID);
}

// N/0->NS/0
//TEST(MapTileLoader, _)
//{
//  const uint64_t TILE_ID = 54;
//  testJunction(TILE_ID);
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  crash_handler::install_crash_handler();

  // NOTE: createMapLayers() requires ros params.
  ros::init(argc, argv, "test_nominal_lane_function", ros::init_options::AnonymousName);
  ros::start();

  return RUN_ALL_TESTS();
}
