import os

from maps.map_layer import MapType
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.feature_layer import FeatureLayer


class MapLayers(object):
    def __init__(self, map_dir=None):
        self.layers = {}
        self.map_dir = map_dir
        self.init_dirs()

    def init_dirs(self):
        if not self.map_dir:
            import rospy
            self.map_dir = rospy.get_param('/maps/map_dir')

    # ----------------------------------------------
    # Main Getter
    # ----------------------------------------------

    def get_layer(self, layer_type, layer_name=''):
        if layer_type == MapType.LANE:
            if MapType.LANE not in self.layers:
                self.layers[MapType.LANE] = self.create_lane_map_layer()
            return self.layers[MapType.LANE]

        elif layer_type == MapType.ROAD:
            if MapType.ROAD not in self.layers:
                self.layers[MapType.ROAD] = self.create_road_graph_layer()
            return self.layers[MapType.ROAD]

        elif layer_type == MapType.DISENGAGE_ZONE:
            if MapType.DISENGAGE_ZONE not in self.layers:
                self.layers[MapType.DISENGAGE_ZONE] = {'single_layers': {}}

            disengage_zones = self.layers[MapType.DISENGAGE_ZONE]

            if layer_name not in disengage_zones['single_layers']:
                fn = os.path.join(self.map_dir, "annotations", "{}.json".format(layer_name))
                disengage_zones['single_layers'][layer_name] = FeatureLayer.from_geojson(fn, layer_type, layer_name)

            return disengage_zones['single_layers'][layer_name]

        elif layer_type == MapType.MAP_READER:
            raise NotImplementedError()

        elif layer_type == MapType.FREE_SPACE:
            raise NotImplementedError()

        elif layer_type == MapType.RADAR_ZONES:
            raise NotImplementedError()

        elif layer_type == MapType.LOCALIZATION_ZONES:
            raise NotImplementedError()

    # ----------------------------------------------
    # Layer Constructors
    # ----------------------------------------------

    def create_lane_map_layer(self, cache_tiles=False, load_tiles=True, fix_dot=True):
        tile_dir = os.path.join(self.map_dir, 'tiles')
        return ConvertedLaneMapLayer(tile_dir, cache_tiles, load_tiles, fix_dot)

    def create_road_graph_layer(self, cache_tiles=False, load_tiles=True):
        tile_dir = os.path.join(self.map_dir, 'road_tiles')
        return GeoJsonTiledMapLayer(tile_dir, ROAD_GRAPH_TILE_LEVEL, cache_tiles, load_tiles, MapType.ROAD)
