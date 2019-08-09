find_package(rostest REQUIRED)

add_rostest_gtest(test_map_tile_loader test_map_tile_loader.test test_map_tile_loader.cpp)
target_link_libraries(test_map_tile_loader ${catkin_LIBRARIES} maps)
