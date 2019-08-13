#! /usr/bin/env python
"""  """
from maps.issue_types import IssueType
from maps.issues import Issue


def lint_junction(junction, lane_map, issue_layer):
    # Extract lanes
    inflows = []
    outflows = []
    for lane_ref in junction.properties['inflow_refs']:
        lane = lane_map.get_tile(lane_ref['tile_id']).get_features('lane')[lane_ref]
        assert lane.properties['direction_of_travel'] == "FORWARD"
        inflows.append(lane)
    for lane_ref in junction.properties['outflow_refs']:
        lane = lane_map.get_tile(lane_ref['tile_id']).get_features('lane')[lane_ref]
        assert lane.properties['direction_of_travel'] == "FORWARD"
        outflows.append(lane)

    if len(outflows) > 1:
        transition_count = {"MERGE": 0, "SPLIT": 0, "UNKNOWN": 0}
        for lane in outflows:
            if lane.properties['from_split'] is True:
                if lane.properties['lane_transition_type'] is None:
                    transition_type = "UNKNOWN"
                else:
                    transition_type = lane.properties['lane_transition_type']
                transition_count[transition_type] += 1

        if transition_count["UNKNOWN"] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.POST_TRANSITION_NO_NORMAL.name))
        elif transition_count["UNKNOWN"] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.POST_TRANSITION_MULTIPLE_NORMAL.name))

    if len(inflows) > 1:
        transition_count = {"MERGE": 0, "SPLIT": 0, "UNKNOWN": 0}
        for lane in inflows:
            if lane.properties['merging'] is True:
                if lane.properties['lane_transition_type'] is None:
                    transition_type = "UNKNOWN"
                else:
                    transition_type = lane.properties['lane_transition_type']
                transition_count[transition_type] += 1

        if transition_count["UNKNOWN"] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.PRE_TRANSITION_NO_NORMAL.name))
        elif transition_count["UNKNOWN"] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.PRE_TRANSITION_MULTIPLE_NORMAL.name))

    return issue_layer
