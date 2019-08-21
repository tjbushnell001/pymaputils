#! /usr/bin/env python
""" Simple script to sift through lanes and verify they follow our constraints on lane labels. """
import argparse
import os
import glob
import rospkg
import maps.routing
import sys

from maps.linting import junction_linter
from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.issues import IssueLayer, IssueLevel, Issue
from maps.issue_types import IssueType
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.utils import routing_utils

parser = argparse.ArgumentParser("Lint all da lanes")
parser.add_argument("--route_id", dest='route_ids', action='append',
                    help="a list of routes to lint")
parser.add_argument("--map_dir", default=None,
                    help="input dir for tiles, default to lane_map_server")
parser.add_argument("--map_reader_dir", default=None,
                    help="dir of map reader files")
parser.add_argument("--out_file", default=None,
                    help="file to write issue layer output")


def lint_route_junctions(route, lane_map, road_map, issue_layer):
    junction_set = set()
    for lane_group in routing_utils.get_lane_groups_in_route(route, road_map, lane_map):
        lane_tile = lane_map.get_tile(lane_group['ref']['tile_id'])
        for lane_segment_ref in lane_group.properties['lane_segment_refs']:
            lane = lane_tile.get_features('lane')[lane_segment_ref]
            if lane.properties['is_emergency_lane']:
                continue
            # Only use starting junctions to avoid duplicate issue checking
            for junction_key in ('start_junction_ref', 'end_junction_ref'):
                junction_ref = lane.properties[junction_key]
                if junction_ref in junction_set:
                    continue
                junction_set.add(junction_ref)

                junction = lane_map.get_feature(junction_ref)
                if junction is None:
                    issue_layer.add_issue(lane, Issue(IssueType.NON_EXISTANT_JUNCTION_REF.name,
                                          msg=str(lane)))
                else:
                    junction_linter.lint_junction(junction, lane_map, issue_layer)


def main():
    args = parser.parse_args()

    map_dir = args.map_dir
    if map_dir is None:
        map_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')

    map_reader_dir = args.map_reader_dir
    if map_reader_dir is None:
        map_reader_dir = os.path.join(rospkg.RosPack().get_path('map_reader'), 'maps')

    routes_dir = os.path.join(rospkg.RosPack().get_path('brain'), 'params/routes')
    route_ids = args.route_ids
    if route_ids is None:
        route_ids = []
        # default to all routes
        for fn in glob.glob(os.path.join(routes_dir, '*.yaml')):
            route_id = os.path.splitext(os.path.basename(fn))[0]
            route_ids.append(route_id)

    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), fix_dot=True)
    road_map = GeoJsonTiledMapLayer(os.path.join(map_dir, 'road_tiles'), tile_level=ROAD_GRAPH_TILE_LEVEL)
    issue_layer = IssueLayer()

    route_failures = 0
    for route_id in route_ids:
        print
        print "*****************************************"
        print "Generating route for [{}]".format(route_id)

        all_waypoints = routing_utils.load_waypoints_from_map_reader(map_reader_dir, route_id)
        routes = maps.routing.find_route(road_map, all_waypoints, routing_utils.Capabilities())

        if routes is None:
            route_failures += 1
            print "Failed to generate route for [{}]".format(route_id)
            continue

        print
        print "Linting route [{}]".format(route_id)

        print
        print 'Linting Junctions'
        lint_route_junctions(routes, lane_map, road_map, issue_layer)

    level_counts = issue_layer.count_issues_by_level()

    print
    print "Num Issues:", issue_layer.count() + route_failures
    if route_failures > 0:
        print '  Route Failures: {}'.format(route_failures)
    for k in sorted(level_counts.keys()):
        print '  {}: {}'.format(k, level_counts[k])

    if args.out_file is not None:
        print
        print "Writing Issues to File [{}]".format(args.out_file)
        issue_layer.write(args.out_file)

    if route_failures > 0 or level_counts[IssueLevel.ERROR] > 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
