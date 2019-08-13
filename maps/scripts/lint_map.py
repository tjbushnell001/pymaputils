#! /usr/bin/env python
""" Simple script to sift through lanes and verify they follow our constraints on lane labels. """
import argparse
import os
import rospkg
import maps.routing

from maps.linting import lane_linter, junction_linter
from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.issues import IssueLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.utils import routing_utils

parser = argparse.ArgumentParser("Lint all da lanes")
parser.add_argument("--route_ids", dest="route_ids", default=None, help="a list of routes to lint, comma separated")
parser.add_argument("--map_dir", dest="map_dir", default=None, help="input dir for tiles, default to lane_map_server")
parser.add_argument("--map_reader_dir", dest="map_reader_dir", default=None, help="dir of map reader files")
parser.add_argument("--out_file", dest="out_file", default=None, help="file to write issue layer output")


def lint_route_junctions(route, lane_map, road_map, issue_layer=None):
    if issue_layer is None:
        issue_layer = IssueLayer()
    junction_set = set()
    for lane_group in routing_utils.get_lane_groups_in_route(route, road_map, lane_map):
        lane_tile = lane_map.get_tile(lane_group['ref']['tile_id'])
        for lane_segment_ref in lane_group.properties['lane_segment_refs']:
            lane = lane_tile.get_features('lane')[lane_segment_ref]
            # Only use starting junctions to avoid duplicate issue checking
            junction_ref = lane.properties['start_junction_ref']
            tile = lane_map.get_tile(junction_ref['tile_id'])
            try:
                junction = tile.get_features('junction')[junction_ref]
            except KeyError:
                # issue_layer.add_issue(lane, Issue(IssueType.NON_EXISTANT_JUNCTION_REF.name))
                continue
            if junction_ref not in junction_set:
                junction_set.add(junction_ref)
                junction_linter.lint_junction(junction, lane_map, issue_layer)
    return issue_layer


def lint_route_lanes(route, lane_map, road_map, issue_layer=None):
    if issue_layer is None:
        issue_layer = IssueLayer()
    for lane_group in routing_utils.get_lane_groups_in_route(route, road_map, lane_map):
        lane_tile = lane_map.get_tile(lane_group.ref['tile_id'])
        for lane_segment_ref in lane_group.properties['lane_segment_refs']:
            lane_linter.lint_lane(lane_tile.get_features('lane')[lane_segment_ref], issue_layer)
    return issue_layer


def main(args):
    lane_map = ConvertedLaneMapLayer(os.path.join(args.map_dir, 'tiles'), fix_dot=True)
    road_map = GeoJsonTiledMapLayer(os.path.join(args.map_dir, 'road_tiles'), tile_level=ROAD_GRAPH_TILE_LEVEL)
    issue_layer = IssueLayer()
    for route_id in args.route_ids:
        print "*****************************************"
        print "Attempting to lint route [{}]".format(route_id)
        print "*****************************************"

        all_waypoints = routing_utils.load_waypoints_from_map_reader(args.map_reader_dir, route_id)
        routes = maps.routing.find_route(road_map, all_waypoints)

        if routes is None:
            print "Failed to generate route for [{}]".format(route_id)
        else:
            # lint_route_lanes(routes, lane_map, road_map, issue_layer)
            print '*****************************************************'
            print 'Finished Lanes, linting junctions'
            print '*****************************************************'
            lint_route_junctions(routes, lane_map, road_map, issue_layer)

    print "Num Issues:", issue_layer.size()
    if args.out_file is not None:
        print "Writing Issues to File [{}]".format(args.out_file)
        issue_layer.write(args.out_file)


if __name__ == '__main__':
    args = parser.parse_args()
    if args.map_dir is None:
        args.map_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')
    if args.map_reader_dir is None:
        args.map_reader_dir = os.path.join(rospkg.RosPack().get_path('map_reader'), 'maps')
    if args.route_ids is None:
        args.route_ids = [f[:-5] for f in os.listdir(args.map_reader_dir) if f.endswith('.json')]
    else:
        args.route_ids = [f for f in args.route_ids.split(",")]

    main(args)
