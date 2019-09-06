import os

from maps.map_layer import MapLayerType
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.single_map_layer import SingleMapLayer


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
        if layer_type == MapLayerType.LANE:
            if MapLayerType.LANE not in self.layers:
                self.layers[MapLayerType.LANE] = self.create_lane_map_layer()
            return self.layers[MapLayerType.LANE]

        elif layer_type == MapLayerType.ROAD:
            if MapLayerType.ROAD not in self.layers:
                self.layers[MapLayerType.ROAD] = self.create_road_graph_layer()
            return self.layers[MapLayerType.ROAD]

        elif layer_type == MapLayerType.DISENGAGE_ZONE:
            if MapLayerType.DISENGAGE_ZONE not in self.layers:
                self.layers[MapLayerType.DISENGAGE_ZONE] = {'single_layers': {}}

            disengage_zones = self.layers[MapLayerType.DISENGAGE_ZONE]

            if layer_name not in disengage_zones['single_layers']:
                fn = os.path.join(self.map_dir, "annotations", "{}.json".format(layer_name))
                disengage_zones['single_layers'][layer_name] = SingleMapLayer(layer_type, layer_name, fn)

            return disengage_zones['single_layers'][layer_name]

        elif layer_type == MapLayerType.MAP_READER:
            raise NotImplementedError()

        elif layer_type == MapLayerType.FREE_SPACE:
            raise NotImplementedError()

        elif layer_type == MapLayerType.RADAR_ZONES:
            raise NotImplementedError()

        elif layer_type == MapLayerType.LOCALIZATION_ZONES:
            raise NotImplementedError()

        elif layer_type == MapLayerType.ISSUE:
            raise NotImplementedError()

    # ----------------------------------------------
    # Layer Constructors
    # ----------------------------------------------

    def create_lane_map_layer(self, cache_tiles=False, load_tiles=True, fix_dot=True):
        tile_dir = os.path.join(self.map_dir, 'tiles')
        return ConvertedLaneMapLayer(tile_dir, cache_tiles, load_tiles, fix_dot)

    def create_road_graph_layer(self, cache_tiles=False, load_tiles=True):
        tile_dir = os.path.join(self.map_dir, 'road_tiles')
        return GeoJsonTiledMapLayer(tile_dir, ROAD_GRAPH_TILE_LEVEL, cache_tiles, load_tiles, MapLayerType.ROAD)
