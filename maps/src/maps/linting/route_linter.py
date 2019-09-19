#! /usr/bin/env python
""" Simple script to sift through lanes and verify they follow our constraints on lane labels. """
import argparse
import os
import maps.routing

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


def lint_route(route, route_id, lane_map, road_map, issue_layer, issue_types=None):
    junction_set = set()

    print "Getting LaneGroups in route: {}".format(route_id)
    route_lane_groups = routing_utils.get_lane_groups_in_route(route, road_map, lane_map)
    print "Linting lane groups..."

    for lane_group in route_lane_groups:
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


def lint_routes(map_dir, map_reader_dir, route_ids, issue_types=None):
    # TODO: Once we make map reader dir its own map layer, pass the layers in instead of the directory locations
    issue_set = set()
    if issue_types is not None:
        for itype in issue_types:
            try:
                issue_set.add(IssueType[itype])
            except KeyError:
                print 'Type [{}] is not a valid IssueType'.format(itype)
                return
    else:
        issue_set = set(IssueType)

    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), fix_dot=True)
    road_graph = GeoJsonTiledMapLayer(os.path.join(map_dir, 'road_tiles'), tile_level=ROAD_GRAPH_TILE_LEVEL)
    issue_layer = IssueLayer()

    failed_routes = set()
    for route_id in route_ids:
        prev_counts = issue_layer.count_issues_by_level()
        route_failed = False

        print
        print "*****************************************"
        print "Generating route for [{}]".format(route_id)

        all_waypoints = routing_utils.load_waypoints_from_map_reader(map_reader_dir, route_id)
        routes = maps.routing.find_route(road_graph, all_waypoints, routing_utils.Capabilities())

        if routes is None:
            route_failed = True
            failed_routes.add(route_id)
            print "Failed to generate route for [{}]".format(route_id)
        else:
            print
            print "Linting route [{}]".format(route_id)

            lint_route(routes, route_id, lane_map, road_graph, issue_layer, issue_set)

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
    num_issues = issue_layer.count() + len(failed_routes)
    print "Total Issues:", num_issues
    print "*****************************************"
    print
    if num_issues == 0:
        print '    None'
    else:
        if len(failed_routes) > 0:
            print '  Route Failures: {}'.format(len(failed_routes))
            for route_id in failed_routes:
                print '    {}'.format(route_id)
            print
        if len(total_counts) == 0:
            print '  Issues:'
            print '    None'
        else:
            for issue_level in sorted(total_counts):
                print '  {}: {}'.format(issue_level.name, total_counts[issue_level])
                for issue_type, count in issue_type_counts[issue_level].iteritems():
                    print '    {} - {}'.format(issue_type, count)
                print

    failures = len(failed_routes) > 0
    for failure_level in (IssueLevel.WARN, IssueLevel.ERROR):
        if failure_level in total_counts and total_counts[failure_level] != 0:
            failures = True

    return issue_layer, failures
