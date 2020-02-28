cmake_minimum_required(VERSION 2.8.3)
project(maps)

find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

# set compiler flags
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_RELEASE "-O2")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CATKIN_COMPONENTS
    geometry_msgs
    std_msgs
    utils_map
    utils_file
    utils_sets
    diagnostics_utils
    )
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES maps
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

include_directories(
  include
  include/${PROJECT_NAME}/utils
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/map_layer.cpp
   src/${PROJECT_NAME}/map_layers.cpp
   src/${PROJECT_NAME}/lane_maps.cpp
   src/${PROJECT_NAME}/road_maps.cpp
   src/${PROJECT_NAME}/geojson_maps.cpp
   src/${PROJECT_NAME}/free_space_maps.cpp
   src/${PROJECT_NAME}/mission.cpp
   src/${PROJECT_NAME}/utils/lane_map_transform.cpp
   src/${PROJECT_NAME}/utils/lane_map_msg_parser.cpp
   src/${PROJECT_NAME}/utils/lane_map_msg_utils.cpp
   src/${PROJECT_NAME}/utils/lane_map_utils.cpp
   src/${PROJECT_NAME}/utils/ego_lane_finder.cpp
   src/${PROJECT_NAME}/utils/lane_annotator.cpp
   src/${PROJECT_NAME}/utils/lane_group_annotator.cpp
   src/${PROJECT_NAME}/utils/lane_map_filtering.cpp
   src/${PROJECT_NAME}/utils/lane_map_following.cpp
   src/${PROJECT_NAME}/utils/map_frame_utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()
