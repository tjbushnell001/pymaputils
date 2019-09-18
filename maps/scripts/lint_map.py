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
parser.add_argument("--route_id", action='append', dest="route_ids", help="a list of routes to lint")
parser.add_argument("--map_dir", default=None, help="input dir for tiles, default to lane_map_server")
parser.add_argument("--map_reader_dir", default=None, help="dir of map reader files")
parser.add_argument("--out_file", default=None, help="file to write issue layer output")
parser.add_argument("--issue_type", dest="issue_types", action='append', default=None,
                    help="list of issue types to search for")

UPDATE_INTERVAL_PERCENT = 20


def lint_route_junctions(route, route_id, lane_map, road_map, issue_layer, issue_types=None):
    junction_set = set()

    print "Getting LaneGroups in Route: {}".format(route_id)

    route_lane_groups = list(routing_utils.get_lane_groups_in_route(route, road_map, lane_map))
    next_interval = 0
    for count, lane_group in enumerate(route_lane_groups, 1):
        percent_complete = 100 * float(count) / float(len(route_lane_groups))
        if percent_complete > next_interval:
            next_interval += UPDATE_INTERVAL_PERCENT
            print "Route: {}, Percent Complete: {}".format(route_id, percent_complete)
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
                    if IssueType.NON_EXISTANT_JUNCTION_REF in issue_types:
                        issue_layer.add_issue(lane, Issue(IssueType.NON_EXISTANT_JUNCTION_REF.name, msg=str(lane)))
                else:
                    junction_linter.lint_junction(junction, lane_map, issue_layer, issue_types)


def main():
    args = parser.parse_args()
    issue_types = set()
    if args.issue_types is not None:
        for itype in args.issue_types:
            try:
                issue_types.add(IssueType[itype])
            except KeyError:
                print 'Type [{}] is not a valid IssueType'.format(itype)
                return
    else:
        issue_types = set(IssueType)

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
        prev_counts = issue_layer.count_issues_by_level()
        route_failed = False

        print
        print "*****************************************"
        print "Generating route for [{}]".format(route_id)

        all_waypoints = routing_utils.load_waypoints_from_map_reader(map_reader_dir, route_id)
        routes = maps.routing.find_route(road_map, all_waypoints, routing_utils.Capabilities())

        if routes is None:
            route_failed = True
            route_failures += 1
            print "Failed to generate route for [{}]".format(route_id)
        else:
            print
            print "Linting route [{}]".format(route_id)

            print
            print 'Linting Junctions'
            lint_route_junctions(routes, route_id, lane_map, road_map, issue_layer, issue_types)

        curr_counts = issue_layer.count_issues_by_level()

        print
        print "Route Issues:", sum(curr_counts.values()) - sum(prev_counts.values()) + int(route_failed)
        if route_failed:
            print '  Route Failures: 1'
        for k in sorted(curr_counts.keys()):
            curr_count = curr_counts[k] - prev_counts.get(k, 0)
            if curr_count > 0:
                print '  {}: {}'.format(k.name, curr_count)

    issue_type_counts = {}
    for issue in issue_layer.get_all_issues():
        if issue.level not in issue_type_counts:
            issue_type_counts[issue.level] = {}
        if issue.issue_type not in issue_type_counts[issue.level]:
            issue_type_counts[issue.level][issue.issue_type] = 0
        issue_type_counts[issue.level][issue.issue_type] += 1

    total_counts = issue_layer.count_issues_by_level()

    print
    print "*****************************************"
    print "Total Issues:", issue_layer.count() + route_failures
    if len(issue_type_counts) == 0:
        print '    None'
    else:
        if route_failures > 0:
            print '  Route Failures: {}'.format(route_failures)
            print
        for issue_level in sorted(total_counts.keys()):
            print '  {}: {}'.format(issue_level.name, total_counts[issue_level])
            for issue_type, count in issue_type_counts[issue_level].iteritems():
                print '    {} - {}'.format(issue_type, count)
            print

    if args.out_file is not None:
        print
        print "Writing Issues to File [{}]".format(args.out_file)
        issue_layer.write(args.out_file)

    if route_failures > 0 or total_counts[IssueLevel.ERROR] > 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
