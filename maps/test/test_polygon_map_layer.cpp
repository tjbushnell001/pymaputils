#include <gtest/gtest.h>

#include "maps/map_layers.h"

const lane_map::PolygonFeature* findFirstMatch(const BoostPoint& point,
                                               const std::vector<lane_map::PolygonFeature>& features) {
  for (const auto& feature : features) {
    if (boost::geometry::within(point, feature.polygon)) {
      return &feature;
    }
  }
  return nullptr;
}

TEST(MapTileLoader, polygonLayersBasics)
{
  
  maps::MapLayers map(maps::createMapLayers({maps::MapLayerType::LOCALIZATION_ZONES}));
  // get layer
  auto layer = map.getLayerAs<maps::PolygonFeatureMapLayer>(maps::MapLayerType::LOCALIZATION_ZONES);
  EXPECT_NE(layer, nullptr);

  EXPECT_NE(layer->getFeatures(), nullptr);
  const std::vector<lane_map::PolygonFeature>& features = *layer->getFeatures();

  EXPECT_EQ(features.size(), 52);

  auto no_zone = findFirstMatch(BoostPoint(37.73301644381975, -122.4041211605072), features);
  EXPECT_EQ(no_zone, nullptr);

  auto reinit_zone = findFirstMatch(BoostPoint(37.73302068638235, -122.40488290786743), features);
  EXPECT_NE(reinit_zone, nullptr);
  EXPECT_EQ(reinit_zone->properties["type"], "reinitialize");

  auto fov_zone = findFirstMatch(BoostPoint(37.73488314788311, -122.40448594093323), features);
  EXPECT_NE(fov_zone, nullptr);
  EXPECT_EQ(fov_zone->properties["type"], "limit_fov");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // NOTE: createMapLayers() requires ros params.
  ros::init(argc, argv, "test_polygon_map_layers", ros::init_options::AnonymousName);
 
  return RUN_ALL_TESTS();
}
