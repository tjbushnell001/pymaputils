import geojson
import os

from collections import OrderedDict
from maps.map_layer import MapLayer
from maps.utils import ref_utils


class FeatureLayer(MapLayer):
    """
    Our custom representation of a geojson FeatureCollection. All of our geojson features have "ref" objects that we
    parse and add a hash functions to. Some map layers are just simple geojson files and exist only as FeatureLayers
    """

    # -----------------------------
    # Constructors
    # -----------------------------

    def __init__(self, layer_type=None, layer_name='', file_path=''):
        """ Initializes a feature layer from a geojson FeatureCollection object. """
        super(FeatureLayer, self).__init__(layer_type, layer_name)
        self.file_path = file_path
        self.collection = None
        self.feature_type_map = {}

    @classmethod
    def from_geojson(cls, file_path, layer_type=None, layer_name=''):
        """ Initializes a feature layer from a geojson file """
        layer = cls(layer_type, layer_name, file_path)
        layer.load()
        return layer

    @classmethod
    def from_collection(cls, feature_collection, layer_type=None, layer_name='', file_path=''):
        assert feature_collection is None or isinstance(feature_collection, geojson.FeatureCollection)
        layer = cls(layer_type, layer_name, file_path)
        layer.collection = feature_collection
        layer._hash_refs()
        return layer

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

    def write(self, file_path):
        with open(file_path, 'w') as f:
            geojson.dump(self.collection, f, sort_keys=True, separators=(',', ':'), indent=1)

    def load(self):
        if not os.path.exists(self.file_path):
            return None
        with open(self.file_path, 'r') as f:
            self.collection = geojson.load(f)
            self._hash_refs()

    # -----------------------------
    # Helper Methods
    # -----------------------------

    def _hash_refs(self):
        for f in self.collection.features:
            if f.feature_type not in self.feature_type_map:
                self.feature_type_map[f.feature_type] = OrderedDict()
            f.ref = ref_utils.hashify(f.ref)
            # We hashify the properties as many features have nested refs
            f.properties = ref_utils.hashify(f.properties)
            self.feature_type_map[f.feature_type][f.ref] = f
