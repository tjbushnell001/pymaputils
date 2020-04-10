""" Library for linting routes. """
import os
import maps.routing

from maps.linting import junction_linter, lane_linter
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.issues import IssueLayer, IssueLevel, Issue
from maps.issue_types import IssueType
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.utils import emblog
from maps.utils import routing_utils


def lint_lane_group(lane_group, lane_map, issue_layer):  # pylint: disable=unused-argument
    # TODO: should this method be refactored to not have unused arg lane_map?

    if len(lane_group.properties['lane_segment_refs']) == 0:
        issue_layer.add_issue(lane_group, Issue(IssueType.NO_LANES_IN_LANE_GROUP.name))


def lint_route(route, route_id, lane_map, road_map, issue_layer):
    junction_set = set()

    print "Getting LaneGroups in route: {}".format(route_id)
    route_lane_groups = routing_utils.get_lane_groups_in_route(route, road_map, lane_map)
    print "Linting lane groups..."

    for lane_group in route_lane_groups:
        # lint the lane group
        lint_lane_group(lane_group, lane_map, issue_layer)

        lane_tile = lane_map.get_tile(lane_group['ref']['tile_id'])
        for lane_segment_ref in lane_group.properties['lane_segment_refs']:
            lane = lane_tile.get_features('lane')[lane_segment_ref]
            if lane.properties['is_emergency_lane']:
                continue

            # lint each lane
            lane_linter.lint_lane(lane, lane_map, issue_layer)

            # Only use starting junctions to avoid duplicate issue checking
            for junction_key in ('start_junction_ref', 'end_junction_ref'):
                junction_ref = lane.properties[junction_key]
                if junction_ref in junction_set:
                    continue
                junction_set.add(junction_ref)

                junction = lane_map.get_feature(junction_ref)
                if junction is None:
                    issue_layer.add_issue(lane, Issue(IssueType.NON_EXISTANT_JUNCTION_REF.name, msg=str(lane)))
                else:
                    junction_linter.lint_junction(junction, lane_map, issue_layer)


def lint_routes(map_dir, map_reader_dir, route_ids, issue_types=None):
    # TODO: Once we make map reader dir its own map layer, pass the layers in instead of the directory locations
    issue_set = None
    if issue_types is not None:
        issue_set = set()
        for itype in issue_types:
            try:
                issue_set.add(IssueType[itype])
            except KeyError:
                print 'Type [{}] is not a valid IssueType'.format(itype)
                return

    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), fix_dot=True)
    road_graph = GeoJsonTiledMapLayer(os.path.join(map_dir, 'road_tiles'), tile_level=ROAD_GRAPH_TILE_LEVEL)
    issue_layer = IssueLayer(filter_types=issue_set)

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

            lint_route(routes, route_id, lane_map, road_graph, issue_layer)

        curr_counts = issue_layer.count_issues_by_level()

        if sum(curr_counts.values()) - sum(prev_counts.values()) + int(route_failed) != 0:
            print
            print "Route Issues:", sum(curr_counts.values()) - sum(prev_counts.values()) + int(route_failed)

        failed = False
        if route_failed:
            emblog.error('  Route Failures: 1')
            failed = True
        for level in sorted(curr_counts.keys(), cmp=IssueLevel.cmp):
            curr_count = curr_counts[level] - prev_counts.get(level, 0)
            if curr_count > 0:
                if level == IssueLevel.ERROR:
                    emblog.error('  {}: {}'.format(level.name, curr_count))
                    failed = True
                else:
                    emblog.info('  {}: {}'.format(level.name, curr_count))

        print
        if failed:
            emblog.error("Failed!")
        else:
            emblog.debug("Passed")

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
            emblog.error('  Route Failures: {}'.format(len(failed_routes)))
            for route_id in failed_routes:
                emblog.error('    {}'.format(route_id))
            print
        if len(total_counts) == 0:
            print '  Issues:'
            print '    None'
        else:
            for issue_level in sorted(total_counts, cmp=IssueLevel.cmp):
                log = emblog.error if issue_level == IssueLevel.ERROR else emblog.info
                log('  {}: {}'.format(issue_level.name, total_counts[issue_level]))
                for issue_type, count in issue_type_counts[issue_level].iteritems():
                    log('    {} - {}'.format(issue_type, count))
                print

    failures = len(failed_routes) > 0
    if total_counts.get(IssueLevel.ERROR, 0) != 0:
        failures = True

    if failures:
        emblog.error("ERRORS:\n")
        for feature_issue in issue_layer.get_all_feature_sets():
            point = feature_issue.point
            for issue in feature_issue.get_issues():
                if issue.level == IssueLevel.ERROR:
                    emblog.error('  {} - Coordinates: ({}, {})'.format(issue, point.y, point.x))

    return issue_layer, failures
