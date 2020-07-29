#! /usr/bin/env python
""" Library to lint map junctions for "Issues" """
from maps.issue_types import IssueType
from maps.issues import Issue
from maps.utils import ref_utils
import geopy

LANE_TRANSITION_TYPES = {"MERGE": "M", "SPLIT": "S", "UNKNOWN": "N"}

def validate_lane_connection(junction, inflow_lane, outflow_lane, issue_layer):
    """
    Adds an issue if the beginning of the new lane is closer to the second last coordinate of the previous lane.
    """
    coordinate_before_end = inflow_lane.geometry.coordinates[-2]
    coordinate_end = inflow_lane.geometry.coordinates[-1]
    coordinate_beginning = outflow_lane.geometry.coordinates[0]

    d_inside_lane = geopy.distance.distance(coordinate_before_end, coordinate_end).meters
    d_outside_lane = geopy.distance.distance(coordinate_before_end, coordinate_beginning).meters

    # This check will be incorrect if the inflow lane and outflow lane have a >90 degree angle. This isn't too much
    # of a concern in our case.
    if d_outside_lane < d_inside_lane:
        issue_layer.add_issue(junction, Issue(IssueType.LANES_OVERLAP), point=None, coordinates=coordinate_end)

def lint_lane_overlap(junction, lane_map, inflows, outflows, issue_layer):
    """
    Checks that the incoming lanes and lane boundaries don't overlap with the outgoing lanes and lane boundaries,
    i.e. that the beginning of the new lane doesn't begin before the previous lane ended.
    """
    if len(inflows) == 0 or len(outflows) == 0:
        return

    inflow_lanes = [lane_map.get_feature(lane_ref) for lane_ref in inflows]
    outflow_lanes = [lane_map.get_feature(lane_ref) for lane_ref in outflows]

    for i in range(len(inflow_lanes)):
        inflow_lane = inflow_lanes[i]
        end_of_lane_coordinate = inflow_lane.geometry.coordinates[-1]

        for j in range(len(outflow_lanes)):
            outflow_lane = outflow_lanes[j]
            validate_lane_connection(junction, inflow_lane, outflow_lane, issue_layer)

            for line_id in ('left', 'right'):
                inflow_boundary_ref = inflow_lane.properties[line_id + '_boundary_ref']
                inflow_boundary = lane_map.get_feature(inflow_boundary_ref)
                outflow_boundary_ref = outflow_lane.properties[line_id + '_boundary_ref']
                outflow_boundary = lane_map.get_feature(outflow_boundary_ref)
                if inflow_boundary is not None and outflow_boundary is not None:
                    validate_lane_connection(junction, inflow_boundary, outflow_boundary, issue_layer)


def lint_junction(junction, lane_map, issue_layer):
    """
    Lint a junction, checking the inflow and outflow lanes for the correct properties and formatting.

    :param junction: a geojson junction object to lint
    :param lane_map: a geojson based tiled map layer
    :param issue_layer: an optional pre initialized issue layer to write new issues to
    :return: the issue layer with any new issues added.
    """
    inflows = junction.properties['inflow_refs']
    outflows = junction.properties['outflow_refs']

    in_transitions = junction_transition_summary(lane_map, inflows)
    out_transitions = junction_transition_summary(lane_map, outflows)
    transition_msg = junction_transition_message(in_transitions, out_transitions)

    lint_lane_overlap(junction, lane_map, inflows, outflows, issue_layer)

    # 1. check that junction has any outflows
    if len(outflows) == 0:
        issue_layer.add_issue(junction, Issue(IssueType.NO_OUTFLOW))

    # 2. check outflow transitions
    for counts in out_transitions.values():
        if sum(counts.values()) > 1 and counts['UNKNOWN'] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.SPLIT_NO_NORMAL, msg=transition_msg))
        elif counts['UNKNOWN'] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.SPLIT_MULTIPLE_NORMAL, msg=transition_msg))

    for lane_ref in outflows:
        lane = lane_map.get_feature(lane_ref)
        if lane.properties['is_emergency_lane']:
            continue
        # 3. check outflow direction of travel
        lane_dot = lane.properties['direction_of_travel']
        if lane_dot != "FORWARD":
            issue_layer.add_issue(lane, Issue(IssueType.INVALID_DIRECTION_OF_TRAVEL, msg=lane_dot))

        # 4. check outflow geometric properties
        if len(outflows) > 1 and not lane.properties['from_split']:
            lane.properties['from_split'] = True
            issue_layer.add_issue(junction, Issue(IssueType.FROM_SPLIT_MISSING_PROPERTY))

    # 5. check inflow transitions
    for counts in in_transitions.values():
        if sum(counts.values()) > 1 and counts['UNKNOWN'] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.MERGE_NO_NORMAL, msg=transition_msg))
        elif counts['UNKNOWN'] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.MERGE_MULTIPLE_NORMAL, msg=transition_msg))

    for lane_ref in inflows:
        lane = lane_map.get_feature(lane_ref)
        if lane.properties['is_emergency_lane']:
            continue
        # 6. check inflow direction of travel
        lane_dot = lane.properties['direction_of_travel']
        if lane_dot != "FORWARD":
            issue_layer.add_issue(lane, Issue(IssueType.INVALID_DIRECTION_OF_TRAVEL, msg=lane_dot))

        # 7. check inflow geometric properties
        if len(inflows) > 1 and not lane.properties['merging']:
            issue_layer.add_issue(junction, Issue(IssueType.MERGING_MISSING_PROPERTY))

    # 8. check junction distance from lanes
    junction_pt = tuple(reversed(junction.geometry['coordinates']))
    max_junction_dist = None

    for lane_refs, pt_idx in [(inflows, -1), (outflows, 0)]:
        for lane_ref in lane_refs:
            lane = lane_map.get_feature(lane_ref)
            if lane.properties['is_emergency_lane']:
                continue
            lane_pt = tuple(reversed(lane.geometry['coordinates'][pt_idx]))

            d = geopy.distance.distance(lane_pt, junction_pt).meters
            if max_junction_dist is None or d > max_junction_dist:
                max_junction_dist = d
    if max_junction_dist is not None and max_junction_dist > 0.10:
        issue_layer.add_issue(junction, Issue(IssueType.JUNCTION_TOO_FAR,
                                              msg="{0:.2f}m".format(max_junction_dist)))

    return issue_layer


def junction_transition_summary(lane_map, lane_refs):
    summary = {
        is_ramp: {
            trans_type: 0
            for trans_type in LANE_TRANSITION_TYPES
        }
        for is_ramp in (False, True)
        }

    for lane_ref in lane_refs:
        lane = lane_map.get_feature(lane_ref)
        if lane.properties['is_emergency_lane']:
            continue
        lane_group_ref = ref_utils.lane_group_ref_from_lane_ref(lane_ref)
        lane_group = lane_map.get_feature(lane_group_ref)

        is_ramp = lane_group.properties['is_ramp'] or False
        assert is_ramp in (True, False)

        transition_type = lane.properties['lane_transition_type'] or "UNKNOWN"
        assert transition_type in LANE_TRANSITION_TYPES

        summary[is_ramp][transition_type] += 1

    return summary


def junction_transition_message(in_summary, out_summary):
    def summary_message(summary):
        parts = []
        for trans_type, display_letter in LANE_TRANSITION_TYPES.iteritems():
            count = summary[trans_type]
            if count > 0:
                if count > 1:
                    parts.append(str(count))
                parts.append(display_letter)
        if len(parts) == 0:
            parts.append('0')
        return "".join(parts)

    return '{}/{} -> {}/{}'.format(summary_message(in_summary[False]),
                                   summary_message(in_summary[True]),
                                   summary_message(out_summary[False]),
                                   summary_message(out_summary[True]))
