import glob
import os
import sys

sys.path.append('..')

from maps import feature_dict
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.map_types import MapType
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL


class MapLayers(object):
    def __init__(self, map_dir=None, free_space_dir=None, radar_zones_dir=None,
                 map_reader_dir=None):
        self.layers = {}

        self._map_dir = map_dir
        self._free_space_dir = free_space_dir
        self._radar_zones_dir = radar_zones_dir
        self._map_reader_dir = map_reader_dir

    # ----------------------------------------------
    # Lazy Load Layer Dirs
    # ----------------------------------------------
    # Dirs are lazy loaded at fetch time to ensure that map layers can be used outside of a ros
    # environment (defaults are not called on unused map layers)

    @property
    def map_dir(self):
        if self._map_dir is None:
            import rospy
            self._map_dir = rospy.get_param('/maps/map_dir')
        return self._map_dir

    @property
    def free_space_dir(self):
        if self._free_space_dir is None:
            import rospy
            self._free_space_dir = rospy.get_param('/maps/free_space_dir')
        return self._free_space_dir

    @property
    def radar_zones_dir(self):
        if self._radar_zones_dir is None:
            import rospy
            self._radar_zones_dir = rospy.get_param('/maps/radar_zones_dir')
        return self._radar_zones_dir

    @property
    def map_reader_dir(self):
        if self._map_reader_dir is None:
            import rospy
            self._map_reader_dir = rospy.get_param('/maps/map_reader_dir')
        return self._map_reader_dir

    @property
    def lidar_lines_dir(self):
        return os.path.join(self.map_dir, 'lidar_lines')

    # ----------------------------------------------
    # Main Getter
    # ----------------------------------------------

    def get_layer(self, layer_type, layer_name='', cache=True, **kwargs):
        if layer_type == MapType.LANE:
            if MapType.LANE not in self.layers or not cache:
                self.layers[MapType.LANE] = self.create_lane_map_layer(**kwargs)
            return self.layers[MapType.LANE]

        elif layer_type == MapType.ROAD:
            if MapType.ROAD not in self.layers or not cache:
                self.layers[MapType.ROAD] = self.create_road_graph_layer(**kwargs)
            return self.layers[MapType.ROAD]

        elif layer_type == MapType.DISENGAGE_ZONE:
            if MapType.DISENGAGE_ZONE not in self.layers or not cache:
                self.layers[MapType.DISENGAGE_ZONE] = self.load_single_layers(
                    self.get_dir(MapType.DISENGAGE_ZONE),
                    spec="disengage_zones*.json")

            return self.layers[MapType.DISENGAGE_ZONE].get(layer_name)

        elif layer_type == MapType.LANE_ANNOTATION:
            if MapType.LANE_ANNOTATION not in self.layers or not cache:
                self.layers[MapType.LANE_ANNOTATION] = self.load_single_layers(
                    self.get_dir(MapType.LANE_ANNOTATION),
                    spec="*.json",
                    as_dict=False)

            return self.layers[MapType.LANE_ANNOTATION].get(layer_name)

        elif layer_type == MapType.MAP_READER:
            if MapType.MAP_READER not in self.layers or not cache:
                self.layers[MapType.MAP_READER] = self.load_single_layers(
                    self.get_dir(MapType.MAP_READER),
                    as_dict=False)
            return self.layers[MapType.MAP_READER].get(layer_name)

        elif layer_type == MapType.FREE_SPACE:
            if MapType.FREE_SPACE not in self.layers or not cache:
                self.layers[MapType.FREE_SPACE] = self.load_single_layers(
                    self.get_dir(MapType.FREE_SPACE),
                    as_dict=False)
            return self.layers[MapType.FREE_SPACE].get(layer_name)

        elif layer_type == MapType.RADAR_ZONE:
            if MapType.RADAR_ZONE not in self.layers or not cache:
                self.layers[MapType.RADAR_ZONE] = self.load_single_layers(
                    self.get_dir(MapType.RADAR_ZONE),
                    as_dict=False)
            return self.layers[MapType.RADAR_ZONE].get(layer_name)

        elif layer_type == MapType.LOCALIZATION_ZONE:
            if MapType.LOCALIZATION_ZONE not in self.layers or not cache:
                self.layers[MapType.LOCALIZATION_ZONE] = feature_dict.load_from_file(
                    self.get_dir(MapType.LOCALIZATION_ZONE), feature_dict=False)

            return self.layers[MapType.LOCALIZATION_ZONE]

        elif layer_type == MapType.LIDAR_LINE:
            if MapType.LIDAR_LINE not in self.layers or not cache:
                self.layers[MapType.LIDAR_LINE] = self.load_single_layers(
                    self.get_dir(MapType.LIDAR_LINE))

            return self.layers[MapType.LIDAR_LINE]

        raise NotImplementedError()

    def get_all_layers(self, layer_type, **kwargs):
        # make sure it gets created
        self.get_layer(layer_type, **kwargs)
        return self.layers[layer_type]

    def get_dir(self, layer_type):
        if layer_type == MapType.LANE:
            return os.path.join(self.map_dir, 'tiles')
        elif layer_type == MapType.ROAD:
            return os.path.join(self.map_dir, 'road_tiles')
        elif layer_type == MapType.DISENGAGE_ZONE:
            return os.path.join(self.map_dir, "annotations")
        elif layer_type == MapType.LANE_ANNOTATION:
            return os.path.join(self.map_dir, "annotations")
        elif layer_type == MapType.MAP_READER:
            return self.map_reader_dir
        elif layer_type == MapType.FREE_SPACE:
            return self.free_space_dir
        elif layer_type == MapType.RADAR_ZONE:
            return self.radar_zones_dir
        elif layer_type == MapType.LOCALIZATION_ZONE:
            return os.path.join(self.map_dir, "../../localization_filter_zones.json")
        elif layer_type == MapType.LIDAR_LINE:
            return self.lidar_lines_dir

    # ----------------------------------------------
    # Layer Constructors
    # ----------------------------------------------

    def create_lane_map_layer(self, cache_tiles=False, load_tiles=True, fix_dot=True):
        tile_dir = self.get_dir(MapType.LANE)
        return ConvertedLaneMapLayer(tile_dir, cache_tiles, load_tiles, fix_dot)

    def create_road_graph_layer(self, cache_tiles=False, load_tiles=True):
        tile_dir = self.get_dir(MapType.ROAD)
        return GeoJsonTiledMapLayer(tile_dir, ROAD_GRAPH_TILE_LEVEL, cache_tiles, load_tiles, MapType.ROAD)

    @staticmethod
    def load_single_layers(map_dir, spec='*.json', as_dict=True):
        # TODO: This loads all data from a layer at once. Add a new class to lazy load different files.
        layers = {}
        for fn in glob.glob(os.path.join(map_dir, spec)):
            layer_name = os.path.splitext(os.path.basename(fn))[0]
            if len(layer_name) == 0:
                continue
            layers[layer_name] = feature_dict.load_from_file(fn, feature_dict=as_dict)
        return layers
