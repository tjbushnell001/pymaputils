#! /usr/bin/env python
import os
import rospy

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.utils import routing_utils
import maps.routing
from perception_msgs.msg import MapTrip
from std_msgs.msg import String
import maps.road_graph
from diagnostics_utils.node_health_publisher import NodeHealthPublisher

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

        self.diagnostics = NodeHealthPublisher('route_generator')

        self.diagnostics.update_status('Route', False, [('Active', False)],
                                       error_message='No Route Requested')

        # Create a subscriber to generate routes
        # TODO(christian): make this a service instead
        self.route_sub = rospy.Subscriber("/route/generate", String, self.generate_route_callback)

    # ------------------------
    # Callbacks
    # ------------------------

    def generate_route_callback(self, route_id):
        rospy.loginfo('Generating route for route_id {}'.format(route_id.data))

        route_ok = False
        route_error = None
        
        # load waypoints from map reader
        all_waypoints = None
        try:
            all_waypoints = routing_utils.load_waypoints_from_map_reader(self.map_reader_dir, route_id.data)
        except IOError as e:
            route_error = "Route doesn't exist for [{}]".format(route_id.data)
            rospy.logerr(route_error)

        routes = None
        if all_waypoints is not None:

            # find the route
            routes = maps.routing.find_route(self.road_graph, all_waypoints,
                                             routing_utils.Capabilities())
            if routes is None:
                route_error = "Failed to generate route for [{}]".format(route_id.data)
                rospy.logerr(route_error)

        if routes is not None:
            route_ok = True
            route_msg = routing_utils.trip_to_msg(routes, all_waypoints)
            route_msg.route_id = route_id.data
            self.route_pub.publish(route_msg)

        # update diagnostics
        self.diagnostics.update_status('Route', route_ok,
                                       [('Active', route_ok),
                                        ('Route Id', route_id.data)],
                                       error_message=route_error)

if __name__ == '__main__':
    rospy.init_node('route_publisher')
    node = RouteGenerationNode()

    # generate initial route
    initial_route = rospy.get_param("/route_id")
    node.generate_route_callback(String(initial_route))

    rospy.loginfo('spinning...')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # publish diagnostics
        node.diagnostics.publish()

        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
