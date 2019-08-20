#! /usr/bin/env python
import os
import rospy

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.utils import routing_utils
import maps.routing
from perception_msgs.msg import MapTrip
from std_msgs.msg import String
import maps.road_graph


class RouteGenerationNode(object):
    def __init__(self):
        """
        Node that converts map reader "routes" into true routes and latches them to the route topic.
        """

        maps_dir = rospy.get_param("/maps/map_dir")
        self.map_reader_dir = rospy.get_param("/maps/map_reader_dir")

        self.road_graph = GeoJsonTiledMapLayer(os.path.join(maps_dir, 'road_tiles'),
                                          maps.road_graph.ROAD_GRAPH_TILE_LEVEL)

        self.route_pub = rospy.Publisher('/route', MapTrip, queue_size=1, latch=True)

        # Create a subscriber to generate routes
        # TODO(christian): make this a service instead
        self.route_sub = rospy.Subscriber("/route/generate", String, self.generate_route_callback)

    # ------------------------
    # Callbacks
    # ------------------------

    def generate_route_callback(self, route_id):
        rospy.loginfo('Generating route for route_id {}'.format(route_id.data))

        # load waypoints from map reader
        all_waypoints = routing_utils.load_waypoints_from_map_reader(self.map_reader_dir, route_id.data)

        # find the route
        routes = maps.routing.find_route(self.road_graph, all_waypoints)

        if routes is None:
            rospy.logerr("Failed to generate route for [{}]".format(route_id.data))
            return

        self.route_pub.publish(routing_utils.trip_to_msg(routes, all_waypoints))


if __name__ == '__main__':
    rospy.init_node('route_publisher')
    node = RouteGenerationNode()

    # generate initial route
    initial_route = rospy.get_param("/route_id")
    node.generate_route_callback(String(initial_route))

    rospy.loginfo('spinning...')
    rospy.spin()
