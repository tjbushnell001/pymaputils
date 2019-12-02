import glob
import json
import os

from maps.utils.lru_cache import LRUCache
from maps.utils import tile_utils


class JsonTiledMapLayer(object):
    """
    Class that represents a tiled map and it's directory structure. No longer loads a single monolithic map. Instead
    this edits individual map files.
    """
    MAX_CACHE_SIZE = 100

    def __init__(self, map_dir, tile_level, cache_tiles=True, load_tiles=True, layer_type=None, json_separators=(',', ': ')):
        """
        :param map_dir: The root directory of this tiled map (usually lives one level below tiled_maps)
        :param cache_tiles: Whether or not to cache tiles
        :param tile_level: the here_maps tile level. This basically defines the resolution of the tiles (larger number
               means smaller tiles).
        :param layer_type: the MapLayer type.
        :param json_seperators: json separators, per json.dump()
        """
        self.map_dir = map_dir
        self.tile_level = tile_level

        self.cache_tiles = cache_tiles
        self.load_tiles = load_tiles
        self.layer_type = layer_type
        self.json_separators = json_separators

        # tile cache
        self.cache = LRUCache(self.MAX_CACHE_SIZE)

    def get_tile(self, tile_id):
        tile = None
        if self.cache_tiles and self.cache.contains(tile_id):
            tile = self.cache.get(tile_id)

        if self.load_tiles and tile is None:
            tile = self.load_tile(tile_id)
            if self.cache_tiles:
                self.cache.set(tile_id, tile)

        return tile

    def add_tile(self, tile_id, tile):
        if self.cache_tiles:
            self.cache.set(tile_id, tile)

    def clear_cache(self):
        self.cache.clear()

    # ---------------------------------------
    # Disk operations to access map files
    # ---------------------------------------
    def load_tile(self, tile_id):
        """
        Load a here map tile and return it.
        :param tile_id: int id of the tile (used to name / fetch json tile files)
        """
        fn = self.get_tile_filename(tile_id)
        if not os.path.exists(fn):
            return None
        with open(fn, 'r') as f:
            tile = json.load(f)
            return tile

    def save_tile(self, tile_id, tile):
        """
        Save a tile to disk in here maps format.
        :param tile_id: id of the tile to fetch
        :param tile: a tile dict in here maps format
        """
        if tile is None:
            return
        self.add_tile(tile_id, tile)

        fn = self.get_tile_filename(tile_id)
        with open(fn, 'w') as f:
            json.dump(tile, f, indent=4, sort_keys=True,
                      separators=self.json_separators)
            print "Saved tile: {}".format(fn)

    def get_tile_list(self):
        """
        Return a list of tile id's that exist in this map.
        """
        return [int(os.path.basename(fn).replace('.json', '')) for fn in
                glob.glob(os.path.join(self.map_dir, '*.json'))]

    # ---------------------------------------
    # Other API Methods
    # ---------------------------------------

    def yield_adjacent_tiles(self, tile_id):
        """
        Yield a tile in the adjacency set of tile with id tile_id.
        :param tile_id: id of tile around which to search
        """
        for adjacent_tile_id in tile_utils.adjacent_tile_ids(tile_id, self.tile_level, include_self=True):
            tile = self.get_tile(adjacent_tile_id)
            if tile is not None:
                yield tile

    def remove_tiles(self):
        """ Remove all tiles on disk. """
        for fn in glob.glob(os.path.join(self.map_dir, '*.json')):
            os.unlink(fn)
        self.cache.clear()

    def tile_exists(self, tile_id):
        """
        Return true if the tile exists
        :param tile_id: The id of the tile to check for
        :return: boolean
        """
        fn = self.get_tile_filename(tile_id)
        return os.path.exists(fn) and os.path.isfile(fn)

    def get_tile_filename(self, tile_id):
        return os.path.join(self.map_dir, '{}.json'.format(tile_id))
