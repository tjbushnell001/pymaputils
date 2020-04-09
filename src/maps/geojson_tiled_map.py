from maps import feature_dict
from maps.tiled_map_layer import JsonTiledMapLayer


class GeoJsonTiledMapLayer(JsonTiledMapLayer):
    def __init__(self, map_dir, tile_level, cache_tiles=True, load_tiles=True, layer_type=None):
        """
        Class that represents a tiled map of GeoJSON tiles.

        :param map_dir: The root directory of this tiled map (usually lives one level below tiled_maps)
        :param tile_level: the here_maps tile level. This basically defines the resolution of the tiles (larger number
            means smaller tiles).
        """
        self.layer_type = layer_type
        super(GeoJsonTiledMapLayer, self).__init__(map_dir, tile_level, cache_tiles=cache_tiles, load_tiles=load_tiles)

    def load_tile(self, tile_id):
        """
        Load a geojson map tile and return it.

        :param tile_id: int id of the tile (used to name / fetch json tile files)
        """
        fn = self.get_tile_filename(tile_id)
        return feature_dict.load_from_file(fn)

    def save_tile(self, tile_id, tile):
        """
        Save a geojson tile to disk.

        :param tile: a tile dict in here maps format
        """
        if tile is None:
            return
        fn = self.get_tile_filename(tile_id)
        feature_dict.write_to_file(fn, tile)
        print "Saved tile: {}".format(fn)

    def get_feature(self, ref):
        tile = self.get_tile(ref['tile_id'])
        if tile is None:
            return None
        return tile.get_feature(ref)
