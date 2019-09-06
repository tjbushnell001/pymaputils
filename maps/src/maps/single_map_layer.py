from maps.map_layer import MapLayer
from maps.utils import geojson_utils


class SingleMapLayer(MapLayer):
    def __init__(self, layer_type, layer_name, file_path):
        super(SingleMapLayer, self).__init__(layer_type, layer_name)
        self.feature_dict = geojson_utils.load_geojson_collection(file_path)

    def get_feature(self, ref):
        return self.feature_dict.get_feature(ref)

    def get_features(self, feature_type):
        return self.feature_dict.get_features(feature_type)
