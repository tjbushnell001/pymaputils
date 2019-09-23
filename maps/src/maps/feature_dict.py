import geojson
import os

from collections import OrderedDict
from maps.utils import ref_utils


class FeatureDict(object):
    """
    Our custom representation of a geojson FeatureCollection. All of our geojson features have "ref" objects that we
    parse and add hash functions to. Some map layers are just simple geojson files and exist only as FeatureDicts
    """

    # -----------------------------
    # Constructor
    # -----------------------------

    def __init__(self, collection):
        """ Initializes a feature dict from a geojson FeatureCollection object. g"""
        self.collection = collection
        self.feature_type_map = {}
        for f in self.collection.features:
            if f.feature_type not in self.feature_type_map:
                self.feature_type_map[f.feature_type] = OrderedDict()
            f.ref = ref_utils.hashify(f.ref)
            # We hashify the properties as many features have nested refs
            f.properties = ref_utils.hashify(f.properties)
            self.feature_type_map[f.feature_type][f.ref] = f

    # -----------------------------
    # Feature Accessor Methods
    # -----------------------------

    @property
    def id(self):
        return self.collection.get('id', None)

    def get_feature(self, ref):
        return self.get_features(ref['type'].replace('_ref', '')).get(ref)

    def get_features(self, feature_type):
        return self.feature_type_map.get(feature_type, {})

    def all_features(self):
        return self.feature_type_map.values()

    def add_feature(self, feature):
        if feature.feature_type not in self.feature_type_map:
            self.feature_type_map[feature.feature_type] = OrderedDict()

        assert feature.ref not in self.feature_type_map[feature.feature_type]

        self.feature_type_map[feature.feature_type][feature.ref] = feature
        self.collection.features.append(feature)

    def delete_feature(self, ref):
        feature_type = ref['type'].replace('_ref', '')
        if feature_type in self.feature_type_map:
            if ref in self.feature_type_map[feature_type]:
                del self.feature_type_map[feature_type][ref]
                if len(self.feature_type_map[feature_type]) == 0:
                    del self.feature_type_map[feature_type]


def load_from_file(file_path):
    """
    Load a geojson file adding our custom ref parsing logic.

    :param file_path: the full file path of the file to load
    :return: a FeatureDict object with fully index-able refs
    """
    if not os.path.exists(file_path):
        return None
    with open(file_path, 'r') as f:
        collection = geojson.load(f)
    return FeatureDict(collection)


def write_to_file(file_path, feature_dict):
    with open(file_path, 'w') as f:
        geojson.dump(feature_dict.collection, f, sort_keys=True, separators=(',', ':'), indent=1)
