import os
import glob

from maps import feature_dict
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.map_types import MapType
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL


class MapLayers(object):
    def __init__(self, map_dir=None, free_space_dir=None, radar_zones_dir=None,
                 map_reader_dir=None):
        self.layers = {}

        self.map_dir = map_dir
        if self.map_dir is None:
            import rospy
            self.map_dir = rospy.get_param('/maps/map_dir')

        self.free_space_dir = free_space_dir
        if self.free_space_dir is None:
            import rospy
            self.free_space_dir = rospy.get_param('/maps/free_space_dir')

        self.radar_zones_dir = radar_zones_dir
        if self.radar_zones_dir is None:
            import rospy
            self.radar_zones_dir = rospy.get_param('/maps/radar_zones_dir')

        self.map_reader_dir = map_reader_dir
        if self.map_reader_dir is None:
            import rospy
            self.map_reader_dir = rospy.get_param('/maps/map_reader_dir')

    # ----------------------------------------------
    # Main Getter
    # ----------------------------------------------

    def get_layer(self, layer_type, layer_name='', **kwargs):
        if layer_type == MapType.LANE:
            if MapType.LANE not in self.layers:
                self.layers[MapType.LANE] = self.create_lane_map_layer(**kwargs)
            return self.layers[MapType.LANE]

        elif layer_type == MapType.ROAD:
            if MapType.ROAD not in self.layers:
                self.layers[MapType.ROAD] = self.create_road_graph_layer(**kwargs)
            return self.layers[MapType.ROAD]

        elif layer_type == MapType.DISENGAGE_ZONE:
            if MapType.DISENGAGE_ZONE not in self.layers:
                self.layers[MapType.DISENGAGE_ZONE] = self.load_single_layers(
                    os.path.join(self.map_dir, "annotations"),
                    spec="disengage_zones*.json")

            return self.layers[MapType.DISENGAGE_ZONE].get(layer_name)

        elif layer_type == MapType.LANE_ANNOTATION:
            if MapType.LANE_ANNOTATION not in self.layers:
                self.layers[MapType.LANE_ANNOTATION] = self.load_single_layers(
                    os.path.join(self.map_dir, "annotations"),
                    spec="*.json",
                    as_dict=False)

            return self.layers[MapType.LANE_ANNOTATION].get(layer_name)

        elif layer_type == MapType.MAP_READER:
            if MapType.MAP_READER not in self.layers:
                self.layers[MapType.MAP_READER] = self.load_single_layers(
                    self.map_reader_dir,
                    as_dict=False)
            return self.layers[MapType.MAP_READER].get(layer_name)

        elif layer_type == MapType.FREE_SPACE:
            if MapType.FREE_SPACE not in self.layers:
                self.layers[MapType.FREE_SPACE] = self.load_single_layers(
                    self.free_space_dir,
                    as_dict=False)
            return self.layers[MapType.FREE_SPACE].get(layer_name)

        elif layer_type == MapType.RADAR_ZONE:
            if MapType.RADAR_ZONE not in self.layers:
                self.layers[MapType.RADAR_ZONE] = self.load_single_layers(
                    self.radar_zones_dir,
                    as_dict=False)
            return self.layers[MapType.RADAR_ZONE].get(layer_name)

        elif layer_type == MapType.LOCALIZATION_ZONE:
            if MapType.LOCALIZATION_ZONE not in self.layers:
                fn = os.path.join(self.map_dir, "../../localization_filter_zones.json")
                self.layers[MapType.LOCALIZATION_ZONE] = feature_dict.load_from_file(fn, feature_dict=False)

            return self.layers[MapType.LOCALIZATION_ZONE]

        raise NotImplementedError()

    def get_all_layers(self, layer_type, **kwargs):
        # make sure it gets created
        self.get_layer(layer_type, **kwargs)
        return self.layers[layer_type]

    # ----------------------------------------------
    # Layer Constructors
    # ----------------------------------------------

    def create_lane_map_layer(self, cache_tiles=False, load_tiles=True, fix_dot=True):
        tile_dir = os.path.join(self.map_dir, 'tiles')
        return ConvertedLaneMapLayer(tile_dir, cache_tiles, load_tiles, fix_dot)

    def create_road_graph_layer(self, cache_tiles=False, load_tiles=True):
        tile_dir = os.path.join(self.map_dir, 'road_tiles')
        return GeoJsonTiledMapLayer(tile_dir, ROAD_GRAPH_TILE_LEVEL, cache_tiles, load_tiles, MapType.ROAD)

    @staticmethod
    def load_single_layers(map_dir, spec='*.json', as_dict=True):
        layers = {}
        for fn in glob.glob(os.path.join(map_dir, spec)):
            layer_name = os.path.splitext(os.path.basename(fn))[0]
            if len(layer_name) == 0:
                continue
            layers[layer_name] = feature_dict.load_from_file(fn, feature_dict=as_dict)
        return layers
