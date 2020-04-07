#include <perception_msgs/Map.h>
#include <perception_msgs/MapBoundary.h>
#include <perception_msgs/MapBoundaryRef.h>
#include <perception_msgs/MapConnector.h>
#include <perception_msgs/MapConnectorRef.h>
#include <perception_msgs/MapJunction.h>
#include <perception_msgs/MapJunctionRef.h>
#include <perception_msgs/MapLane.h>
#include <perception_msgs/MapLaneGroup.h>
#include <perception_msgs/MapLaneGroupRef.h>
#include <perception_msgs/MapTile.h>
#include "maps/map_layers.h"
#include "utils/map/lane_map_structs.h"

namespace map_msg_utils {

perception_msgs::MapBoundary getBoundaryMsg(const lane_map::Boundary& boundary);

perception_msgs::MapBoundaryRef getBoundaryRefMsg(const lane_map::BoundaryRef& boundary_ref);

perception_msgs::MapLane getLaneMsg(const lane_map::Lane& lane);

perception_msgs::MapLaneRef getLaneRefMsg(const lane_map::LaneRef& lane_ref);

perception_msgs::MapLaneGroup getLaneGroupMsg(const lane_map::LaneGroup& lane_group);

perception_msgs::MapLaneGroupRef getLaneGroupRefMsg(const lane_map::LaneGroupRef& lg_ref);

perception_msgs::MapJunction getJunctionMsg(const lane_map::Junction& junction);

perception_msgs::MapJunctionRef getJunctionRefMsg(const lane_map::JunctionRef& junc_ref);

perception_msgs::MapConnector getConnectorMsg(const lane_map::Connector& connector);

perception_msgs::MapConnectorRef getConnectorRefMsg(const lane_map::ConnectorRef& conn_ref);

perception_msgs::MapTile getTileMsg(const lane_map::Tile& tile);

perception_msgs::Map getMapMsg(const maps::LaneSubMap& map);
}
