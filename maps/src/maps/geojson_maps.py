import geojson
import os

from maps.tiled_map_layer import JsonTiledMapLayer
from maps.utils import geojson_utils
from collections import OrderedDict


class TileDict(object):
    def __init__(self, tile):
        self.tile = tile
        assert tile is None or isinstance(tile, geojson.FeatureCollection)
        self.features = {}
        for f in tile.features:
            if f.feature_type not in self.features:
                self.features[f.feature_type] = OrderedDict()
            f.ref = geojson_utils.hashify(f.ref)
            f.properties = geojson_utils.hashify(f.properties)
            self.features[f.feature_type][f.ref] = f

    def get_features(self, feature_type):
        return self.features.get(feature_type, {})

    def add_feature(self, feature):
        if feature.feature_type not in self.features:
            self.features[feature.feature_type] = OrderedDict()

        assert feature.ref not in self.features[feature.feature_type]

        self.features[feature.feature_type][feature.ref] = feature
        self.tile.features.append(feature)

    # --------------
    # Properties
    # --------------

    @property
    def id(self):
        return self.tile.get('id', None)


class GeoJsonTiledMapLayer(JsonTiledMapLayer):
    """
    Class that represents a tiled map of GeoJSON tiles

    """
    def __init__(self, map_dir, tile_level, cache_tiles=True, load_tiles=True):
        """
        :param map_dir: The root directory of this tiled map (usually lives one level below tiled_maps)
        :param tile_level: the here_maps tile level. This basically defines the resolution of the tiles (larger number
            means smaller tiles).
        """
        super(GeoJsonTiledMapLayer, self).__init__(map_dir, tile_level, cache_tiles=cache_tiles, load_tiles=load_tiles)

    def load_tile(self, tile_id):
        """
        Load a geojson map tile and return it in here_maps format.

        :param tile_id: int id of the tile (used to name / fetch json tile files)
        """
        fn = self.get_tile_filename(tile_id)
        if not os.path.exists(fn):
            return None
        with open(fn, 'r') as f:
            tile = TileDict(geojson.load(f))
            return tile

    def save_tile(self, tile_id, tile):
        """
        Save a geojson tile to disk.

        :param tile: a tile dict in here maps format
        """
        if tile is None:
            return

        fn = self.get_tile_filename(tile_id)
        with open(fn, 'w') as f:
            geojson.dump(tile.tile, f, sort_keys=True, separators=(',', ':'), indent=1)
            print "Saved tile: {}".format(fn)

    def get_feature(self, ref):
        tile = self.get_tile(ref['tile_id'])
        if tile is None:
            return None
        feature_type = ref['type'].replace('_ref', '')
        return tile.get_features(feature_type).get(ref)
