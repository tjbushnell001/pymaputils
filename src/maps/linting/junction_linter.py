#! /usr/bin/env python
""" Library to lint map junctions for "Issues" """
from maps.issue_types import IssueType
from maps.issues import Issue
from maps.utils import ref_utils
import geopy

LANE_TRANSITION_TYPES = {"MERGE": "M", "SPLIT": "S", "UNKNOWN": "N"}


def lint_junction(junction, lane_map, issue_layer):
    """
    Lint a junction, checking the inflow and outflow lanes for the correct properties and formatting.

    :param junction: a geojson junction object to lint
    :param lane_map: a geojson based tiled map layer
    :param issue_layer: a pre initialized issue layer to write new issues to
    :return: the issue layer with any new issues added.
    """
    inflows = junction.properties['inflow_refs']
    outflows = junction.properties['outflow_refs']

    in_transitions = junction_transition_summary(lane_map, inflows)
    out_transitions = junction_transition_summary(lane_map, outflows)
    transition_msg = junction_transition_message(in_transitions, out_transitions)

    # 1. check that junction has any outflows
    if len(outflows) == 0:
        issue_layer.add_issue(junction, Issue(IssueType.NO_OUTFLOW))
        if len(inflows) == 0:
            issue_layer.add_issue(junction, Issue(IssueType.ORPHANED_JUNCTION))

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

def lint_junction_for_orphan(junction, issue_layer):
    """
    Lint a junction on an emergency lane,
    checking the inflow and outflow lanes for the correct properties and formatting.

    :param junction: a geojson junction object to lint
    :param issue_layer: a pre initialized issue layer to write new issues to
    :return: the issue layer with any new issues added.
    """
    inflows = junction.properties['inflow_refs']
    outflows = junction.properties['outflow_refs']

    if len(outflows) == 0 and len(inflows) == 0:
        issue_layer.add_issue(junction, Issue(IssueType.ORPHANED_JUNCTION))

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
