#! /usr/bin/env python
import geojson
import os

from enum import Enum
from maps.utils import geojson_utils


class WaypointType(Enum):
    TRIP_ORIGIN = 'trip_origin'
    TRIP_DESTINATION = 'trip_destination'
    REROUTE_ORIGIN = 'reroute_origin'
    ROUTE_POINT = 'route_point'
    SUB_DESTINATION = 'sub_destination'
    SUB_ORIGIN = 'sub_origin'
    ALLOWED_RAMP = 'allowed_ramp'


class Capabilities(object):
    def __init__(self):
        self.whitelist_ramps = True


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
    waypoint_types = set(waypoint_type.value for waypoint_type in set(WaypointType))
    for point_idx, point in enumerate(map_reader['features']):
        assert point.is_valid

        if "waypoint" not in point.properties:
            continue
        assert point.properties['waypoint'] in waypoint_types
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
