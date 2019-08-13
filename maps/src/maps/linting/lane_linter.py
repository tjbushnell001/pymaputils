
#! /usr/bin/env python
"""  """
from maps.issue_types import IssueType
from maps.issues import Issue, IssueLevel

TRANSITION_KEYS = ['from_split', 'upcoming_split', 'merged', 'merging']


def lint_lane(lane, issue_layer):
    transition_props = [1 for k in TRANSITION_KEYS if lane.properties[k]]
    valid = len(transition_props) <= 1

    if not valid:
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
