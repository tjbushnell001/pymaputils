#include <perception_msgs/Map.h>
#include <perception_msgs/MapBoundary.h>
#include <perception_msgs/MapConnector.h>
#include <perception_msgs/MapConnectorRef.h>
#include <perception_msgs/MapJunction.h>
#include <perception_msgs/MapLane.h>
#include <perception_msgs/MapLaneGroup.h>
#include <perception_msgs/MapLaneGroupRef.h>
#include <perception_msgs/MapTile.h>
#include "utils/map/structs.h"

namespace map_msg_utils {

perception_msgs::MapBoundary getBoundaryMsg(const map_utils::Boundary& boundary);

perception_msgs::MapLane getLaneMsg(const map_utils::Lane& lane);

perception_msgs::MapLaneGroup getLaneGroupMsg(const map_utils::LaneGroup& lane_group);

perception_msgs::MapLaneGroupRef getLaneGroupRefMsg(const map_utils::LaneGroupRef& lg_ref);

perception_msgs::MapJunction getJunctionMsg(const map_utils::Junction& junction);

perception_msgs::MapConnector getConnectorMsg(const map_utils::Connector& connector);

perception_msgs::MapConnectorRef getConnectorRefMsg(const map_utils::ConnectorRef& conn_ref);

perception_msgs::MapTile getTileMsg(const map_utils::Tile& tile);

perception_msgs::Map getMapMsg(const map_utils::Map& map_struct);
}
