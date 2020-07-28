import geojson
import json
import shapely.geometry as sg

from collections import Counter
from enum import Enum
from maps.utils import geojson_utils
from maps.utils import ref_utils
from maps.issue_types import IssueType


class IssueLevel(Enum):
    WARN = 0
    ERROR = 1
    IGNORE = 2

    @staticmethod
    def cmp(issue1, issue2):
        return cmp(issue1.value, issue2.value)


class IssueLayer(object):
    """
    A set of features (i.e. FeatureIssueSet) and their associated issues.
    TODO(christian): Make this a GeoJsonMapLayer
    """

    def __init__(self, filter_types=None):
        """
        Arguments:
        filter_types: (optional) Only accept issues of the given type (iterable
                      of IssueType).
        """

        self.features = {}

        self.filter_types = None
        if filter_types is not None:
            self.filter_types = set(filter_types)

    def add_issue(self, feature, issue, point=None, coordinates=None):
        if (self.filter_types is not None and
                issue.issue_type not in self.filter_types):
            # we're filtering and this isn't a support issue type
            return

        issue_set = self.get_feature_issues(feature, create=True)

        if coordinates is not None:
            point = sg.asShape(feature.geometry).representative_point()

        if point is not None:
            issue_set.update_point(point)

        issue_set.add_issue(issue)

    def add_ignore(self, feature, issue_type):
        self.get_feature_issues(feature, create=True).add_ignore(issue_type)

    def remove_ignore(self, feature, issue_type):
        feature_issues = self.get_feature_issues(feature, create=False)
        if feature_issues is not None:
            feature_issues.add_ignore(issue_type)

    def get_all_issues(self):
        issue_set = set()
        for feature in self.features.values():
            for issue in feature.get_issues():
                issue_set.add(issue)
        return issue_set

    def get_feature_issues(self, feature, create=False):
        issue_set = self.features.get(feature.ref)
        if issue_set is None and create:
            self.features[feature.ref] = issue_set = FeatureIssueSet.from_feature(feature)
        return issue_set

    def has_issues(self, feature):
        return feature.ref in self.features

    def get_all_feature_sets(self):
        return self.features.itervalues()

    def count(self):
        num_issues = 0
        for issue_set in self.features.itervalues():
            num_issues += len(issue_set.issues)

        return num_issues

    def count_issues_by_level(self):
        num_issues = Counter()
        for issue_set in self.features.itervalues():
            for issue in issue_set.issues.itervalues():
                num_issues[issue.level] += 1

        return num_issues

    def write(self, fp, include_ignores=False):
        """
        Write the issues layer to a json file in geojson.
        :param fp: A file path to write to
        :param include_ignores: whether to include ignored issues in the file
        """
        feature_collection = {'type': 'FeatureCollection', 'features': []}
        for issue_set in self.features.values():
            for ref, issue in issue_set.issues.iteritems():
                if issue.level != IssueLevel.IGNORE or include_ignores:
                    feature_collection['features'].append(convert_issue_to_geojson(issue, ref, issue_set))
        with open(fp, 'w') as f:
            json.dump(feature_collection, f, indent=4, separators=(',', ':'))

    def load(self, fp):
        """
        Read a set of issues from an issues file.
        :param fp: a file path to read from
        :return:
        """
        with open(fp, 'r') as f:
            raw_issues = geojson.load(f)
        for feature in raw_issues.features:
            issue, ref, point = parse_geojson_issue(feature)
            if ref not in self.features:
                self.features[ref] = FeatureIssueSet(ref, point)
            self.features[ref].add_issue(issue)


class FeatureIssueSet(object):
    """ The set of all issues associated with a feature. """

    def __init__(self, feature_ref, point):
        self.point = point
        self.feature_ref = feature_ref
        self.issues = {}
        self.ignore_issues = set()

    @classmethod
    def from_feature(cls, feature):
        point = sg.asShape(feature.geometry).representative_point()
        issue_set = cls(feature.ref, point)
        for issue_type_name in feature.properties.get('ignore_issues', []):
            issue_set.add_ignore(IssueType[issue_type_name])

        return issue_set

    def update_point(self, point):
        self.point = point

    def add_ignore(self, issue_type):
        self.ignore_issues.add(issue_type)

    def remove_ignore(self, issue_type):
        if issue_type in self.ignore_issues:
            self.ignore_issues.remove(issue_type)

    def add_issue(self, issue):
        if issue.issue_type in self.ignore_issues:
            issue.level = IssueLevel.IGNORE
        ref = ref_utils.hashify({'feature_ref': self.feature_ref, 'type': issue.issue_type.value})
        if ref not in self.issues:
            self.issues[ref] = issue
        elif issue.level.value >= self.issues[ref].level.value:
            self.issues[ref] = issue

    def get_issues(self):
        return self.issues.values()


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


def parse_geojson_issue(geojson_issue):
    issue = Issue(
        issue_type=geojson_issue.properties['type'],
        level=IssueLevel[geojson_issue.properties['level']],
        msg=geojson_issue.properties['msg']
    )
    ref = ref_utils.hashify(geojson_issue.ref['feature_ref'])
    point = geojson_issue.geometry['coordinates']
    return issue, ref, point
