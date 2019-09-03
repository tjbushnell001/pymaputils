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

auto const JUNCTION_CONNECTOR_ID = 1;
auto const JUNCTION_ID = 1;

auto const IN_LANE_LG_ID = 1;
auto const IN_LANE_ID = 1;

auto const OUT_LANE_LG_ID = 100;
auto const OUT_LANE_ID = 1;

const std::shared_ptr<maps::LaneSubMap> getLaneSubMapFromTile(uint64_t tile_id) {
  maps::MapLayers map(maps::createMapLayers({maps::MapLayerType::LANE}));

  // get lane map
  auto lane_map = map.getLayerAs<maps::LaneMapLayer>(maps::MapLayerType::LANE);
  EXPECT_NE(lane_map, nullptr);

  lane_map->loadTiles({tile_id});

  auto lane_sub_map = lane_map->getSubMap();
  EXPECT_NE(lane_sub_map->getTile(tile_id), nullptr);

  return lane_sub_map;
}

void testJunction(uint64_t tile_id, const int valid_in_lanes, const int valid_out_lanes) {
    const auto lane_sub_map = getLaneSubMapFromTile(tile_id);

    const auto junction_ref = lane_map::JunctionRef(tile_id, JUNCTION_CONNECTOR_ID, JUNCTION_ID);
    auto junction = lane_sub_map->getJunction(junction_ref);
    EXPECT_NE(junction, nullptr);

    lane_map::LaneRef answer_lane_ref;
    const auto in_nominal_lanes = ego_lane_finder::getNominalLanes(*lane_sub_map, junction->inflow_refs, lane_map_utils::TraverseDirection::IN);

    // Assert the size is num_valid_lanes
    EXPECT_EQ(in_nominal_lanes.size(), valid_in_lanes);
    if (in_nominal_lanes.size() > 0) {
        answer_lane_ref = lane_map::LaneRef(tile_id, IN_LANE_LG_ID, IN_LANE_ID);
        // In addition, the "first" in lane (i.e. the far left lane) is still taken to be the answer in ambiguous cases
        EXPECT_EQ(in_nominal_lanes[0], answer_lane_ref);
    }

    const auto out_nominal_lanes = ego_lane_finder::getNominalLanes(*lane_sub_map, junction->outflow_refs, lane_map_utils::TraverseDirection::OUT);

    // Assert the size is valid_out_lanes
    EXPECT_EQ(out_nominal_lanes.size(), valid_out_lanes);
    if (out_nominal_lanes.size() > 0) {
        answer_lane_ref = lane_map::LaneRef(tile_id, OUT_LANE_LG_ID, OUT_LANE_ID);
        // In addition, the "first" out lane (i.e. the far left lane) is still taken to be the answer in ambiguous cases
        EXPECT_EQ(out_nominal_lanes[0], answer_lane_ref);
    }
}


// Standard case, normal lane to normal lane connection
// N/0 -> N/0
TEST(MapTileLoader, _N_0__N_0)
{
  const uint64_t tile_id = 1;
  testJunction(tile_id, 1, 1);
}

// Standard ramp exit
// N/0 -> N/N
TEST(MapTileLoader, _N_0__N_N)
{
  const uint64_t tile_id = 2;
  testJunction(tile_id, 1, 1);
}

// Standard ramp entrance to normal lane
// 0/N->N/0
TEST(MapTileLoader, _0_N__N_0)
{
  const uint64_t tile_id = 3;
  testJunction(tile_id, 1, 1);
}

// Highway to exit ramp
// N/0->0/N
TEST(MapTileLoader, _N_0__0_N)
{
  const uint64_t tile_id = 4;
  testJunction(tile_id, 1, 1);
}

// Standard ramp merging onto highway
// N/N->N/0
TEST(MapTileLoader, _N_N__N_0)
{
  const uint64_t tile_id = 5;
  testJunction(tile_id, 1, 1);
}

// Highway merge, no ramp
// NM/0->N/0
TEST(MapTileLoader, _NM_0__N_0)
{
  const uint64_t tile_id = 6;
  testJunction(tile_id, 1, 1);
}

// Nominal ramp lane continuation
// 0/N->0/N
TEST(MapTileLoader, _0_N__0_N)
{
  const uint64_t tile_id = 7;
  testJunction(tile_id, 1, 1);
}

// Lane transitioning to pre-merge lane
// N/0->M/0
TEST(MapTileLoader, _N_0__M_0)
{
  const uint64_t tile_id = 8;
  testJunction(tile_id, 1, 1);
}

// Ramp lane to highway merge lane (preparing to merge on highway)
// 0/N->M/0
TEST(MapTileLoader, _0_N__M_0)
{
  const uint64_t tile_id = 9;
  testJunction(tile_id, 1, 1);
}

// Transition to normal lane post split
// S/0->N/0
TEST(MapTileLoader, _S_0__N_0)
{
  const uint64_t tile_id = 10;
  testJunction(tile_id, 1, 1);
}

// Nominal highway split lane
// N/0->NS/0
TEST(MapTileLoader, _N_0__NS_0)
{
  const uint64_t tile_id = 11;
  testJunction(tile_id, 1, 1);
}

// Invalid highway split, returns two possible outflows
// N/0->2N/0
TEST(MapTileLoader, _N_0__2N_0)
{
  const uint64_t tile_id = 12;
  testJunction(tile_id, 1, 2);
}

// Invalid! Merge returns two possible inflow lanes
// 2N/0->N/0
TEST(MapTileLoader, _2N_0__N_0)
{
  const uint64_t tile_id = 13;
  testJunction(tile_id, 2, 1);
}

// Lane ends without a merge. Invalid! This should not happen on the road. Still we assert the return is what we expect
// N/0->0/0
TEST(MapTileLoader, _N_0__0_0)
{
  const uint64_t tile_id = 14;
  testJunction(tile_id, 1, 0);
}

// Split lane is turning into a ramp
// S/0->0/N
TEST(MapTileLoader, _S_0__0_N)
{
  const uint64_t tile_id = 15;
  testJunction(tile_id, 1, 1);
}

// Normal hw lane is splitting on an exit ramp
// N/0->0/NS
TEST(MapTileLoader, _N_0__0_NS)
{
  const uint64_t tile_id = 16;
  testJunction(tile_id, 1, 1);
}

// Ramp lane is splitting into the highway as well as continuing as a ramp
// 0/N->N/N
TEST(MapTileLoader, _0_N__N_N)
{
  const uint64_t tile_id = 17;
  testJunction(tile_id, 1, 1);
}

// Ramp lanes are merging together in a highway lane
// 0/NM->N/0
TEST(MapTileLoader, _0_NM__N_0)
{
  const uint64_t tile_id = 18;
  testJunction(tile_id, 1, 1);
}

// Ramp lanes are merging together
// 0/NM->0/N
TEST(MapTileLoader, _0_NM__0_N)
{
  const uint64_t tile_id = 19;
  testJunction(tile_id, 1, 1);
}

// Lane begins without a split. This is allowed but happens very infrequently
// 0/0->N/0
TEST(MapTileLoader, _0_0__N_0)
{
  const uint64_t tile_id = 20;
  testJunction(tile_id, 0, 1);
}

// Lanes split on a ramp without a valid transition label. Invalid! Still assert the return type.
// 0/N->0/2N
TEST(MapTileLoader, _0_N__0_2N)
{
  const uint64_t tile_id = 21;
  testJunction(tile_id, 1, 2);
}

// Split lane turns to normal lane on a ramp
// 0/S->0/N
TEST(MapTileLoader, _0_S__0_N)
{
  const uint64_t tile_id = 22;
  testJunction(tile_id, 1, 1);
}

// Normal lane transitions to pre-merge lane on a ramp
// 0/N->0/M
TEST(MapTileLoader, _0_N__0_M)
{
  const uint64_t tile_id = 23;
  testJunction(tile_id, 1, 1);
}

// Two normal lanes merge on a ramp without a valid transition label. Invalid! Still assert the return type.
// 0/2N->0/N
TEST(MapTileLoader, _0_2N__0_N)
{
  const uint64_t tile_id = 24;
  testJunction(tile_id, 2, 1);
}

// Lane merges on highway while simultaneously splitting onto a ramp. Rare but valid.
// NM/0->N/N
TEST(MapTileLoader, _NM_0__N_N)
{
  const uint64_t tile_id = 25;
  testJunction(tile_id, 1, 1);
}

// Nominal split on a ramp
// 0/N->0/NS
TEST(MapTileLoader, _0_N__0_NS)
{
  const uint64_t tile_id = 26;
  testJunction(tile_id, 1, 1);
}

// Nominal 3 way split on the highway
// N/0->N2S/0
TEST(MapTileLoader, _N_0__N2S_0)
{
  const uint64_t tile_id = 27;
  testJunction(tile_id, 1, 1);
}

// Lane merges on highway while simultaneously splitting again on the highway.
// NM/0->NS/0
TEST(MapTileLoader, _NM_0__NS_0)
{
  const uint64_t tile_id = 28;
  testJunction(tile_id, 1, 1);
}

// 3 way lane split on highway, 2 lanes go to ramp.
// N/0->N/NS
TEST(MapTileLoader, _N_0__N_NS)
{
  const uint64_t tile_id = 29;
  testJunction(tile_id, 1, 1);
}

// 3 way lane merge on incoming ramp, 2 lanes from ramp to highway
// N/NM->N/0
TEST(MapTileLoader, _N_NM__N_0)
{
  const uint64_t tile_id = 30;
  testJunction(tile_id, 1, 1);
}

// Same as case 20 except it happens on a ramp.
// 0/0->0/N
TEST(MapTileLoader, _0_0__0_N)
{
  const uint64_t tile_id = 31;
  testJunction(tile_id, 0, 1);
}

// Same as case 14 except it happens on a ramp
// 0/N->0/0
TEST(MapTileLoader, _0_N__0_0)
{
  const uint64_t tile_id = 32;
  testJunction(tile_id, 1, 0);
}

// Nominal triple merge case on the highway
// N2M/0->N/0
TEST(MapTileLoader, _N2M_0__N_0)
{
  const uint64_t tile_id = 33;
  testJunction(tile_id, 1, 1);
}

// Two lanes merge on a ramp into another merge lane, lacking proper transition labels. Invalid! Still assert return
// values.
// 0/2N->0/M
TEST(MapTileLoader, _0_2N__0_M)
{
  const uint64_t tile_id = 34;
  testJunction(tile_id, 2, 1);
}

// Ramp and highway lanes merge into a split lane. Not valid from a semantics perspective, but unambiguous
// traversal wise
// N/N->S/0
TEST(MapTileLoader, _N_N__S_0)
{
  const uint64_t tile_id = 35;
  testJunction(tile_id, 1, 1);
}

// Recently split lane converts to upcoming merge lane.
// S/0->M/0
TEST(MapTileLoader, _S_0__M_0)
{
  const uint64_t tile_id = 36;
  testJunction(tile_id, 1, 1);
}

// Normal ramp lane transitions to split ramp lane. Not valid from a semantics perspective, but unambiguous
// traversal wise
// 0/N->0/S
TEST(MapTileLoader, _0_N__0_S)
{
  const uint64_t tile_id = 37;
  testJunction(tile_id, 1, 1);
}

// Merge lane transitions to standard lane. Not valid from a semantics perspective, but unambiguous
// traversal wise
// M/0->N/0
TEST(MapTileLoader, _M_0__N_0)
{
  const uint64_t tile_id = 38;
  testJunction(tile_id, 1, 1);
}

// 3 way split lacking proper transition labels. Invalid! Assert return values
// N/0->2NS/0
TEST(MapTileLoader, _N_0__2NS_0)
{
  const uint64_t tile_id = 39;
  testJunction(tile_id, 1, 2);
}

// Repeat of case 13 but with more lane groups. Invalid!
// 2N/0->N/0
TEST(MapTileLoader, _2N_0__N_0_TWO_IN_LANE_GROUPS)
{
  const uint64_t tile_id = 40;
  testJunction(tile_id, 2, 1);
}

// Merge transitions into ramp lane
// NM/0->0/N
TEST(MapTileLoader, _NM_0__0_N)
{
  const uint64_t tile_id = 41;
  testJunction(tile_id, 1, 1);
}

// Ramp and highway lanes merge
// N/N->M/0
TEST(MapTileLoader, _N_N__M_0)
{
  const uint64_t tile_id = 42;
  testJunction(tile_id, 1, 1);
}

// Standard split case with an extra highway ramp
// N/0->NS/N
TEST(MapTileLoader, _N_0__NS_N)
{
  const uint64_t tile_id = 43;
  testJunction(tile_id, 1, 1);
}

// 3 way merge on ramp into highway lane
// 0/N2M->N/0
TEST(MapTileLoader, _0_N2M__N_0)
{
  const uint64_t tile_id = 44;
  testJunction(tile_id, 1, 1);
}

// Same as case 6 except with two incoming lane groups
// NM/0->N/0
TEST(MapTileLoader, _NM_0__N_0_TWO_IN_LANE_GROUPS)
{
  const uint64_t tile_id = 45;
  testJunction(tile_id, 1, 1);
}

// Same as case 31 except that it is a merge lane that begins
// 0/0->0/M
TEST(MapTileLoader, _0_0__0_M)
{
  const uint64_t tile_id = 46;
  testJunction(tile_id, 0, 1);
}

// Post split ramp lane turns into standard highway lane
// 0/S->N/0
TEST(MapTileLoader, _0_S__N_0)
{
  const uint64_t tile_id = 47;
  testJunction(tile_id, 1, 1);
}

// 3 way merge, 2 lanes on highway, none on a ramp
// NM/N->N/0
TEST(MapTileLoader, _NM_N__N_0)
{
  const uint64_t tile_id = 48;
  testJunction(tile_id, 1, 1);
}

// Standard split, one highway one ramp lane
// N/0->N/S
TEST(MapTileLoader, _N_0__N_S)
{
  const uint64_t tile_id = 49;
  testJunction(tile_id, 1, 1);
}

// 3 way merge immediately into a lane split
// NM/N->NS/0
TEST(MapTileLoader, _NM_N__NS_0)
{
  const uint64_t tile_id = 50;
  testJunction(tile_id, 1, 1);
}

// Same as case 20 except that it's a merge lane
// 0/0->M/0
TEST(MapTileLoader, _0_0__M_0)
{
  const uint64_t tile_id = 51;
  testJunction(tile_id, 0, 1);
}

// 4 way split, invalid transition labels. Invalid! Still assert return values.
// N/0->2NS/N
TEST(MapTileLoader, _N_0__2NS_N)
{
  const uint64_t tile_id = 52;
  testJunction(tile_id, 1, 2);
}

// Simultaneous merge and split from ramp and back to a ramp
// N/N->N/N
TEST(MapTileLoader, _N_N__N_N)
{
  const uint64_t tile_id = 53;
  testJunction(tile_id, 1, 1);
}

// Same as case 11 but with 2 outgoing lane groups
// N/0->NS/0
TEST(MapTileLoader, _N_0__NS_0_TWO_OUT_LANE_GROUPS)
{
  const uint64_t tile_id = 54;
  testJunction(tile_id, 1, 1);
}

/***********************************************************************************************************
 * IMPORTANT: If you want to add new test cases, add them below and start the tiles at
 * tile # 55+
 ***********************************************************************************************************/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  crash_handler::install_crash_handler();

  // NOTE: createMapLayers() requires ros params.
  ros::init(argc, argv, "test_nominal_lane_function", ros::init_options::AnonymousName);
  ros::start();

  return RUN_ALL_TESTS();
}
