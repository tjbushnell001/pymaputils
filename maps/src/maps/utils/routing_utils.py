#! /usr/bin/env python
import geojson
import os
import rospy

from geometry_msgs.msg import Point
from perception_msgs.msg import MapWaypoint, MapRoute, MapTrip, MapRoadSegmentRef
from maps.utils import geojson_utils

WAYPOINT_TYPES = {
    'trip_origin': MapWaypoint.WAYPOINT_TRIP_ORIGIN,
    'trip_destination': MapWaypoint.WAYPOINT_TRIP_DESTINATION,
    'reroute_origin': MapWaypoint.WAYPOINT_REROUTE_ORIGIN,
    'route_point': MapWaypoint.WAYPOINT_ROUTE_POINT,
    'sub_destination': MapWaypoint.WAYPOINT_SUB_DESTINATION,
    'sub_origin': MapWaypoint.WAYPOINT_SUB_ORIGIN,
    'allowed_ramp': MapWaypoint.WAYPOINT_ALLOWED_RAMP,
}

class Capabilities(object):
    def __init__(self):
        self.whitelist_ramps = True

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


def waypoint_to_msg(wp):
    """
    Convert a waypoint to a MapWaypoint message for publishing.

    :param wp: a geojson feature describing a waypoint on the map
    :return: a MapWaypoint ROS message
    """
    coordinates = wp.geometry['coordinates']
    msg = MapWaypoint(
        id=wp.ref,
        waypoint_type=WAYPOINT_TYPES[wp.properties['waypoint_type']],
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


def create_route_feature(road_graph, road_seg_refs, route_wp_ids):
    """
    Converts a route (i.e. list of road segments) into a geojson feature for visualization.

    :param road_graph: the road graph tiled map layer
    :param road_seg_refs: the list of road segments in this route
    :param route_wp_ids: the list of route waypoint id's that make up this route
    :return: a geojson Feature
    """
    left_boundary = []
    right_boundary = []
    for ref in road_seg_refs:
        ref_tile = road_graph.get_tile(ref['tile_id'])
        utm_zone = ref_tile.collection.properties['utm_zone']
        utm_lat_band = ref_tile.collection.properties['utm_lat_band']

        road_seg = ref_tile.get_features('road_segment')[ref]
        left_line = geojson_utils.downsample_line(road_seg.properties['left_boundary'],
                                                  utm_zone, utm_lat_band,
                                                  tolerance=5.0,
                                                  min_points=10)

        right_line = geojson_utils.downsample_line(road_seg.properties['right_boundary'],
                                                   utm_zone, utm_lat_band,
                                                   tolerance=5.0,
                                                   min_points=10)
        left_boundary.extend(left_line)
        right_boundary.extend(right_line)

    boundary = geojson_utils.boundaries_to_poly(left_boundary, right_boundary)
    route = geojson_utils.create_feature(
        'route',
        None,
        boundary,
        waypoint_refs=route_wp_ids,
        road_segment_refs=road_seg_refs)
    return route


def load_waypoints_from_map_reader(map_reader_dir, route_id):
    """
    Load the map reader data

    :param map_reader_dir: directory where map reader info is stored
    :param route_id: the "id" of the route (i.e. the name of the route file)
    :return: a list of geojson features representing the waypoints from the map reader route
    """
    map_reader_fn = os.path.join(map_reader_dir, route_id + '.json')
    map_reader = geojson.load(open(map_reader_fn, 'r'))

    return create_waypoints_from_map_reader(map_reader, route_id)


def create_waypoints_from_map_reader(map_reader, route_id):
    """
    Create a list of geojson Features for visualizing waypoints.

    :param map_reader: a geojson feature collection of waypoints
    :param route_id: the route "id" (i.e. the name of the file)
    :return: a list of geojson Features
    """
    waypoints = []
    wp_id = 0
    for point_idx, point in enumerate(map_reader['features']):
        assert point.is_valid

        if "waypoint" not in point.properties:
            continue
        assert point.properties['waypoint'] in WAYPOINT_TYPES
        wp = geojson_utils.create_feature(
            'waypoint',
            wp_id,
            point.geometry,
            waypoint_type=point.properties['waypoint'],
            source='map reader {} {}'.format(route_id, point_idx))

        wp_id += 1

        waypoints.append(wp)

    return waypoints


def get_lane_groups_in_route(route, road_map, lane_map):
    """
    Yield each lane group in a route.

    :param route: a route generated using routing.find_route
    :param road_map: a tiled map layer containing road tiles
    :param lane_map: a tiled map layer containing lane tiles
    :return: yield a set of lane groups
    """
    for route_road_refs, _ in route:
        for road_ref in route_road_refs:
            road_tile = road_map.get_tile(road_ref['tile_id'])
            road_segment = road_tile.get_features('road_segment')[road_ref]
            for lane_group_ref in road_segment.properties['lane_group_refs']:
                lane_tile = lane_map.get_tile(lane_group_ref['tile_id'])
                lane_group = lane_tile.get_features('lane_group')[lane_group_ref]
                yield lane_group
