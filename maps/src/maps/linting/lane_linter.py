""" Library for linting lanes. """

import utm
import math
import shapely.geometry

from maps.issues import IssueLevel, Issue
from maps.issue_types import IssueType
from maps.utils import tile_utils

def line_to_utm(geometry, utm_zone=None):
    """ Creates a shapely LineString in utm """
    points = geometry['coordinates']

    return shapely.geometry.LineString(
        (utm.from_latlon(p[1], p[0], force_zone_number=utm_zone)[:2]
         for p in points))

def check_line(line, feature, line_name, issue_layer):
    """ Check basic properties of a line """
    # 1. isn't empty
    if line.is_empty:
        issue_layer.add_issue(feature, Issue(IssueType.EMPTY_LINE.name,
                                             msg="empty {}".format(line_name)))
    # 2. doesn't cross itself
    if not line.is_simple:
        msg = "{} intersects itself".format(line_name)
        issue_layer.add_issue(feature, Issue(IssueType.SELF_CROSSING_LINE.name,
                                             msg=msg))

def norm_angle(a):
    """ Nomalize an angle +-pi (in radians) """
    return (a + math.pi) % (2 * math.pi) - math.pi

def line_theta(line, index=0):
    """ Find the heading at the given index of a line """
    p1 = line.coords[index]
    p2 = line.coords[index + 1]
    return norm_angle(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

def line_relative_theta(center_line, other_line):
    """ Find the heading difference between two lines """

    center_heading = line_theta(center_line)

    other_heading = line_theta(other_line)

    return norm_angle(other_heading - center_heading)

def lint_lane(lane, lane_map, issue_layer):
    if lane.properties['is_emergency_lane']:
        return

    # use a single utm zone, so all points are in the same zone at utm boundaries
    tile_bounds = tile_utils.tile_bounds(lane.ref['tile_id'], lane_map.tile_level)
    utm_zone = utm.from_latlon(tile_bounds[0], tile_bounds[1])[2]

    transition_type = lane.properties['lane_transition_type'] or "UNKNOWN"

    # check basic line issues
    center_line = line_to_utm(lane.geometry, utm_zone=utm_zone)

    check_line(center_line, lane, 'center line', issue_layer)

    for line_id in ('left', 'right'):
        line_name = line_id + ' line'
        boundary_ref = lane.properties[line_id + '_boundary_ref']
        boundary = lane_map.get_feature(boundary_ref)
        if boundary is None:
            issue_layer.add_issue(lane, Issue(IssueType.MISSING_BOUNDARY.name, msg=str(boundary_ref)))
            continue

        boundary_line = line_to_utm(boundary.geometry, utm_zone=utm_zone)

        # check basic line issues
        check_line(boundary_line, lane, line_name, issue_layer)

        # check boundaries are headed same direction as center
        lh = line_relative_theta(center_line, boundary_line)
        if abs(lh) > math.pi / 2:
            msg = "{} goes wrong direction ({:0.1f} degrees)".format(line_name, math.degrees(lh))
            issue_layer.add_issue(lane, Issue(IssueType.WRONG_DIRECTION_LINE.name,
                                              level=IssueLevel.WARN,
                                              msg=msg))


        # checkout boundaries don't cross center
        # NOTE: Only check this on NORMAL lanes ("UNKNOWN") because merge and
        # split lanes cross their center lines.
        if transition_type == "UNKNOWN" and boundary_line.intersects(center_line):
            msg = "{} crosses center".format(line_name)
            issue_layer.add_issue(lane, Issue(IssueType.LINE_CROSSES_CENTER.name,
                                              level=IssueLevel.WARN,
                                              msg=msg))
