#! /usr/bin/env python
""" Library to lint map junctions for "Issues" """
from maps.issue_types import IssueType
from maps.issues import Issue, IssueLayer


def lint_junction(junction, lane_map, issue_layer=None):
    """
    Lint a junction, checking the inflow and outflow lanes for the correct properties and formatting.

    :param junction: a geojson junction object to lint
    :param lane_map: a geojson based tiled map layer
    :param issue_layer: an optional pre initialized issue layer to write new issues to
    :return: the issue layer with any new issues added.
    """
    issue_layer = IssueLayer() if issue_layer is None else issue_layer

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
            if lane.properties['lane_transition_type'] is None:
                transition_type = "UNKNOWN"
            else:
                transition_type = lane.properties['lane_transition_type']
            transition_count[transition_type] += 1
            if lane.properties['from_split'] is False:
                issue_layer.add_issue(junction, Issue(IssueType.FROM_SPLIT_MISSING_PROPERTY.name))

        if transition_count["UNKNOWN"] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.SPLIT_NO_NORMAL.name))
        elif transition_count["UNKNOWN"] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.SPLIT_MULTIPLE_NORMAL.name))

    if len(inflows) > 1:
        transition_count = {"MERGE": 0, "SPLIT": 0, "UNKNOWN": 0}
        for lane in inflows:
            if lane.properties['lane_transition_type'] is None:
                transition_type = "UNKNOWN"
            else:
                transition_type = lane.properties['lane_transition_type']
            transition_count[transition_type] += 1
            if lane.properties['merging'] is False:
                issue_layer.add_issue(junction, Issue(IssueType.MERGING_MISSING_PROPERTY.name))

        if transition_count["UNKNOWN"] == 0:
            issue_layer.add_issue(junction, Issue(IssueType.MERGE_NO_NORMAL.name))
        elif transition_count["UNKNOWN"] > 1:
            issue_layer.add_issue(junction, Issue(IssueType.MERGE_MULTIPLE_NORMAL.name))

    if len(inflows) > 1 and len(outflows) > 1:
        issue_layer.add_issue(junction, Issue(IssueType.MULTIPLE_INFLOW_OUTFLOWS.name))

    return issue_layer
