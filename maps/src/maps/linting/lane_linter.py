#! /usr/bin/env python
""" Library to lint map lanes for "Issues" """
from maps.issue_types import IssueType
from maps.issues import Issue, IssueLayer, IssueLevel

TRANSITION_KEYS = ['from_split', 'upcoming_split', 'merged', 'merging']


def lint_lane(lane, issue_layer=None):
    """
    Lint a lane, checking transitioning properties, warning if there are more than one property true at a time.

    :param lane: a geojson lane object to lint
    :param issue_layer: an optional pre initialized issue layer to write new issues to
    :return: the issue layer with any new issues added.
    """
    issue_layer = IssueLayer() if issue_layer is None else issue_layer
    transition_props = [k for k in TRANSITION_KEYS if lane.properties[k]]

    if len(transition_props) > 1:
        issue_layer.add_issue(feature=lane, issue=Issue(
            issue_type=IssueType.MULTIPLE_TRANSITION_PROPERTIES.name,
            level=IssueLevel.WARN,
            msg="from_split: {}, upcoming_split: {}, merged: {}, merging: {}".format(
                lane.properties['from_split'],
                lane.properties['upcoming_split'],
                lane.properties['merged'],
                lane.properties['merging']
            )
        ))
    return issue_layer
