import os
import rospy

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from enum import Enum


class MapLayerType(Enum):
    LANE = "LANE"
    LANE_ANNOTATION = "LANE_ANNOTATION"
    ROAD = "ROAD"
    MAP_READER = "MAP_READER"
    FREE_SPACE = "FREE_SPACE"
    RADAR_ZONES = "RADAR_ZONES"
    LOCALIZATION_ZONES = "LOCALIZATION_ZONES"
    ISSUE = "ISSUE"


class MapLayer(object):
    def __init__(self, layer_type, layer_name=''):
        self.layer_type = layer_type
        self.layer_name = layer_name


class MapLayers(object):
    def __init__(self):
        self.layers = {}

    # TODO: add a config method to change directory base?

    def get_layer(self, layer_type):
        if layer_type == MapLayerType.LANE:
            if MapLayerType.LANE not in self.layers:
                self.layers[MapLayerType.LANE] = self.create_lane_map_layer()
            return self.layers[MapLayerType.LANE]

        elif layer_type == MapLayerType.ROAD:
            if MapLayerType.ROAD not in self.layers:
                self.layers[MapLayerType.ROAD] = self.create_road_graph_layer()
            return self.layers[MapLayerType.ROAD]

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

    @staticmethod
    def create_lane_map_layer(cache_tiles=False, load_tiles=True, fix_dot=True):
        tile_dir = os.path.join(rospy.get_param('/maps/map_dir'), 'tiles')
        return ConvertedLaneMapLayer(tile_dir, cache_tiles, load_tiles, fix_dot)

    @staticmethod
    def create_road_graph_layer(cache_tiles=False, load_tiles=True):
        tile_dir = os.path.join(rospy.get_param('/maps/map_dir'), 'road_tiles')
        return GeoJsonTiledMapLayer(tile_dir, ROAD_GRAPH_TILE_LEVEL, cache_tiles, load_tiles)
