#! /usr/bin/env python
""" Utils around constructing ROS Routing messages. Split out from routing_utils to remove the dependency on rospy."""
import rospy

from geometry_msgs.msg import Point
from perception_msgs.msg import MapWaypoint, MapRoute, MapTrip, MapRoadSegmentRef
from maps.utils.routing_utils import WaypointType

WAYPOINT_TYPES_MAP = {
    WaypointType.TRIP_ORIGIN: MapWaypoint.WAYPOINT_TRIP_ORIGIN,
    WaypointType.TRIP_DESTINATION: MapWaypoint.WAYPOINT_TRIP_DESTINATION,
    WaypointType.REROUTE_ORIGIN: MapWaypoint.WAYPOINT_REROUTE_ORIGIN,
    WaypointType.ROUTE_POINT: MapWaypoint.WAYPOINT_ROUTE_POINT,
    WaypointType.SUB_DESTINATION: MapWaypoint.WAYPOINT_SUB_DESTINATION,
    WaypointType.SUB_ORIGIN: MapWaypoint.WAYPOINT_SUB_ORIGIN,
    WaypointType.ALLOWED_RAMP: MapWaypoint.WAYPOINT_ALLOWED_RAMP,
}


def trip_to_msg(routes, all_waypoints):
    """
    Convert a set of routes and their waypoints to a MapTrip ros message for publishing.

    :param routes: the set of routes
    :param all_waypoints: the set of waypoints
    :return: a perception_msgs MapTrip message wrapping the routes and waypoints
    """
    wp_by_id = {wp.ref: wp for wp in all_waypoints}
    msg = MapTrip(
        routes=[route_to_msg(r, w, wp_by_id) for r, w in routes],
        waypoints=[waypoint_to_msg(w) for w in all_waypoints],
    )
    msg.header.stamp = rospy.Time.now()
    return msg


def trip_to_geometry(road_graph, routes):
    """
    Convert a set of routes to a polygon surrounding the entire route.

    :param road graph map
    :param routes: the set of routes
    :return: (lat, lon) points wrapping the routes
    """

    left_path = []
    right_path = []

    for route,_ in routes:
        for road_ref in route:
            road_seg = road_graph.get_feature(road_ref)
            assert road_seg is not None

            left_path.extend(road_seg.properties['left_boundary'])
            right_path.extend(road_seg.properties['right_boundary'])

    right_path.reverse()
    points = left_path + right_path
    return points


def waypoint_to_msg(wp):
    """
    Convert a waypoint to a MapWaypoint message for publishing.

    :param wp: a geojson feature describing a waypoint on the map
    :return: a MapWaypoint ROS message
    """
    coordinates = wp.geometry['coordinates']
    msg = MapWaypoint(
        id=wp.ref,
        waypoint_type=WAYPOINT_TYPES_MAP[WaypointType[wp.properties['waypoint_type']]],
        source=wp.properties['source'])

    # position as a gps fix
    msg.gps_position.latitude = coordinates[1]
    msg.gps_position.longitude = coordinates[0]

    # position as a point
    msg.position.x = coordinates[0]
    msg.position.y = coordinates[1]

    return msg


def route_to_msg(road_seg_refs, route_wp_ids, all_waypoints):
    """
    Converts a route (i.e. list of road segment id's) and an associated waypoint list into a MapRoute message.

    :param road_seg_refs: a list of road segment refs
    :param route_wp_ids: a list of waypoints along this route used to define the geometry
    :param all_waypoints: a set of all waypoints used to look up waypoints by their ids.
    :return: a MapRoute message
    """
    msg = MapRoute(
        waypoint_refs=route_wp_ids,
        road_segment_refs=[MapRoadSegmentRef(tile_id=r['tile_id'], id=r['id'])
                           for r in road_seg_refs]
    )

    # add all waypoints as multipoint geometry
    for wp_id in route_wp_ids:
        wp = all_waypoints[wp_id]
        p = wp.geometry.__geo_interface__["coordinates"]
        msg.geometry.points.append(Point(x=p[0], y=p[1]))

    return msg
