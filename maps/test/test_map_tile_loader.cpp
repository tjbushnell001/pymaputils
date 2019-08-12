#include <gtest/gtest.h>

#include "maps/map_layers.h"

std::vector<maps::MapLayerType> ALL_LAYER_TYPES = {
  maps::MapLayerType::LANE, maps::MapLayerType::LANE_ANNOTATION, maps::MapLayerType::ROAD,
  maps::MapLayerType::MAP_READER, maps::MapLayerType::FREE_SPACE
};

TEST(MapTileLoader, mapLayersBasics)
{
  const std::vector<std::string> layer_names = { "foo", "baz", "bar", "" };
  const std::vector<std::string> bad_names = { "doh", "wtf" };

  maps::MapLayers map;
  for (auto layer_type : ALL_LAYER_TYPES) {
    for (const auto& layer_name : layer_names) {
      // hasLayer()
      EXPECT_FALSE(map.hasLayer(layer_type, layer_name));
      // generic getLayer()
      EXPECT_EQ(map.getLayer(layer_type, layer_name), nullptr);
    }
  }

  for (auto layer_type : ALL_LAYER_TYPES) {
    for (const auto& layer_name : layer_names) {
      const std::string fn = "";
      auto layer = std::make_shared<maps::GeoJsonMapLayer>(layer_type, layer_name, fn);
      map.addLayer(std::static_pointer_cast<maps::MapLayer>(layer));
    }
  }

  for (auto layer_type : ALL_LAYER_TYPES) {
    for (const auto& layer_name : layer_names) {
      // hasLayer()
      EXPECT_TRUE(map.hasLayer(layer_type, layer_name));
      // generic getLayer()
      EXPECT_NE(map.getLayer(layer_type, layer_name), nullptr);
      // getLayerAs()
      EXPECT_NE(map.getLayerAs<maps::GeoJsonMapLayer>(layer_type, layer_name), nullptr);
    }
  }
  // make sure layers we didn't define are still undefined
  for (auto layer_type : ALL_LAYER_TYPES) {
    for (const auto& layer_name : bad_names) {
      // hasLayer()
      EXPECT_FALSE(map.hasLayer(layer_type, layer_name));
      // generic getLayer()
      EXPECT_EQ(map.getLayer(layer_type, layer_name), nullptr);
      // getLayerAs()
      EXPECT_EQ(map.getLayerAs<maps::GeoJsonMapLayer>(layer_type, layer_name), nullptr);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // NOTE: createMapLayers() requires ros params. If used, add ros::init()

  return RUN_ALL_TESTS();
}
