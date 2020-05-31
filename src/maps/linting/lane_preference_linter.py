import itertools

import utm
import shapely.geometry as sg

from maps.utils.geojson_utils import geojson_polygon_to_shapely_utm
from maps.utils import lane_map_utils
from maps.issues import Issue
from maps.issue_types import IssueType


def lint_route_preferences(lane_map, route_lane_groups, lane_preference_layer, issue_layer):
    """

    :param route_lane_groups: a list of lane groups
    :param lane_preference_layer: A FeatureDict with the lane preferences in it
    :param issue_layer: an IssueLayer to add issues to
    :return: None
    """
    for lane_preference_polygon in lane_preference_layer.get_features('lane_preference_polygon').itervalues():
        lane_preference_intersections = 0
        rep_point = sg.asShape(lane_preference_polygon['geometry']).representative_point()
        reference_utm_zone = utm.latlon_to_zone_number(rep_point.y, rep_point.x)
        polygon_feature_border_utm = geojson_polygon_to_shapely_utm(lane_preference_polygon['geometry'],
                                                                    reference_utm_zone)
        for lane_group in route_lane_groups:
            lane_group_border_utm = geojson_polygon_to_shapely_utm(lane_group['geometry'],
                                                                   reference_utm_zone)

            if polygon_feature_border_utm.intersects(lane_group_border_utm):
                lane_preference_intersections += 1
                check_preference_lanes_valid(lane_map, lane_group, lane_preference_polygon, issue_layer)

        if lane_preference_intersections == 0:
            issue_layer.add_issue(lane_preference_polygon,
                                  Issue(IssueType.POLYGON_INTERSECTS_WITH_NO_LANE_GROUPS,
                                        msg=("Polygon {} in route {} doesn't intersect"
                                             "with any lane groups.").format(
                                            lane_preference_polygon['ref']['id'],
                                            lane_preference_polygon['ref']['route_id'])))


def check_preference_lanes_valid(lane_map, lane_group, lane_preference_polygon, issue_layer):
    """
    Checks if the lanes asked for in preferences exist in the lane group.
    Should usually be used *after* checking if lane_group and lane_preference_polygon intersect, not before.
    :param lane_group: the lane group to check
    :param lane_preference_polygon: a GeoJson polygon to check
    :param issue_layer:
    :return: None
    """
    preferred_lanes = lane_preference_polygon['properties'].get('preferred_lanes', [])
    lanes_to_avoid = lane_preference_polygon['properties'].get('lanes_to_avoid', [])
    tile = lane_map.get_tile(lane_group['ref']['tile_id'])
    n_lanes = len(lane_map_utils.non_emergency_lanes(lane_group, lane_map))
    lane_group_id = lane_group['ref']['id']
    polygon_id = lane_preference_polygon['ref']['id']
    verbose = False
    for lane_num in itertools.chain(preferred_lanes, lanes_to_avoid):
        if not 1 <= lane_num <= n_lanes:
            message = ("Lane {} in polygon {} in route {} not valid. "
                       "There are not that many lanes in lane group {}").format(
                lane_num, lane_preference_polygon['ref']['id'],
                lane_preference_polygon['ref']['route_id'], lane_group['ref'])

            issue_layer.add_issue(lane_preference_polygon,
                                  Issue(IssueType.PREFERENCE_LANE_NOT_IN_LANE_GROUP, msg=message))
