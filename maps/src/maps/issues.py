import geojson
import json
import shapely.geometry as sg

from collections import defaultdict
from enum import Enum
from maps.utils import geojson_utils


class IssueLevel(Enum):
    IGNORE = "IGNORE"
    WARN = "WARN"
    ERROR = "ERROR"


class IssueLayer(object):
    """
    A set of features (i.e. FeatureIssueSet) and their associated issues.
    TODO(christian): Make this a GeoJsonMapLayer
    """
    def __init__(self):
        self.features = {}

    def issue_ignored(self, ref, ignore_key):
        issue_set = self.features.get(ref)
        if issue_set is None:
            return False
        return ignore_key in issue_set.ignore_issues

    def add_issue(self, feature, issue, point=None):
        issue_set = self.get_feature_issues(feature, create=True)

        if point is not None:
            issue_set.update_point(point)
            del point

        issue_set.add_issue(issue)

    def get_feature_issues(self, feature, create=False):
        issue_set = self.features.get(feature.ref)
        if issue_set is None and create:
            self.features[feature.ref] = issue_set = FeatureIssueSet(feature)
        return issue_set

    def has_issues(self, feature):
        return feature.ref in self.features

    def get_all_features(self):
        return self.features.itervalues()

    def size(self):
        num_issues = 0
        for issue_set in self.features.values():
            num_issues += len(issue_set.issues)
        return num_issues

    def write(self, fn):
        """
        Write the issues layer to a json file in geojson.
        :param fn: A file location where to write
        """
        feature_collection = {'type': 'FeatureCollection', 'features': []}
        for issue_set in self.features.values():
            for ref, issue in issue_set.issues.iteritems():
                if issue not in issue_set.ignore_issues:
                    feature_collection['features'].append(convert_issue_to_geojson(issue, ref, issue_set))
        with open(fn, 'w') as f:
            json.dump(feature_collection, f, indent=4)


class FeatureIssueSet(object):
    """ The set of all issues associated with a feature. """
    def __init__(self, feature):
        self.id = (feature.feature_type, feature.ref)
        self.geom_type = feature.geometry['type']

        if self.geom_type == 'LineString':
            self.point = sg.LineString(feature.geometry['coordinates']).representative_point()
        elif self.geom_type == 'Point':
            self.point = sg.Point(feature.geometry['coordinates']).representative_point()
        elif self.geom_type == 'Polygon':
            self.point = sg.Polygon(feature.geometry['coordinates']).representative_point()
        else:
            raise NotImplementedError("Geometry type {} not supported in Issues yet".format(feature.geometry['type']))

        self.issues = {}
        self.ignore_issues = set()

    def update_point(self, point):
        self.point = point

    def add_ignore(self, ignore_issues):
        self.ignore_issues.update(ignore_issues)

    def add_issue(self, issue):
        ref = geojson_utils.hashify({'feature_ref': self.id[1], 'type': issue.issue_type})
        if ref not in self.issues:
            self.issues[ref] = issue

    def get_issues(self):
        return self.issues

    def get_issue_strings(self):
        level_counts = defaultdict(lambda: 0)
        for issue in self.issues.values():

            level_counts[issue.level] += 1
            key = "{}-{}".format(issue.level.value.lower(),
                                 level_counts[issue.level])
            value = issue.issue_type
            if issue.msg != "":
                value += ": " + issue.details
            yield key, value


class Issue(object):
    """ A base "issue" object. This basically holds the type, a message describing the issue and it's severity. """
    def __init__(self, issue_type, level=IssueLevel.ERROR, msg=""):
        """
        :param issue_type: a string id
        :param level: an IssueLevel enum
        :param msg: an optional descriptive message
        """
        self.issue_type = issue_type
        self.msg = msg
        self.level = level

    def __repr__(self):
        return "{}: {}{}".format(self.level, self.issue_type, ": " + self.msg if self.msg else "")


# ----------------------------------
# Issue utility methods
# ----------------------------------

def convert_issue_to_geojson(issue, ref, issue_set):
    return geojson_utils.create_feature(
        feature_type='issue',
        feature_ref=ref,
        type=issue.issue_type,
        geometry=geojson.Point(*issue_set.point.coords),
        msg=issue.msg,
        level=issue.level.name)
