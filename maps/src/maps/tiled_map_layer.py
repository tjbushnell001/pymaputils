import collections
import glob
import json
import os

from maps.map_layer import MapLayer
from maps.utils import tile_utils


class LRUCache(object):
    """ Elegant implementation from https://www.kunxi.org/2014/05/lru-cache-in-python/ """
    def __init__(self, capacity):
        self.capacity = capacity
        self.cache = collections.OrderedDict()

    def get(self, key):
        try:
            value = self.cache.pop(key)
            self.cache[key] = value
            return value
        except KeyError:
            return None

    def set(self, key, value):
        try:
            self.cache.pop(key)
        except KeyError:
            if len(self.cache) >= self.capacity:
                self.cache.popitem(last=False)
        self.cache[key] = value

    def contains(self, key):
        return key in self.cache

    def clear(self):
        self.cache = collections.OrderedDict()


class JsonTiledMapLayer(MapLayer):
    """
    Class that represents a tiled map and it's directory structure. No longer loads a single monolithic map. Instead
    this edits individual map files.
    """
    MAX_CACHE_SIZE = 100

    def __init__(self, layer_type, layer_name, map_dir, tile_level, cache_tiles=True, load_tiles=True):
        """
        :param map_dir: The root directory of this tiled map (usually lives one level below tiled_maps)
        :param cache_tiles: Whether or not to cache tiles
        :param tile_level: the here_maps tile level. This basically defines the resolution of the tiles (larger number
            means smaller tiles).
        """
        super(JsonTiledMapLayer, self).__init__(layer_type, layer_name)
        self.map_dir = map_dir
        self.tile_level = tile_level

        self.cache_tiles = cache_tiles
        self.load_tiles = load_tiles

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
            json.dump(tile, f, indent=4, sort_keys=True)
            print "Saved tile: {}".format(fn)

    def get_tile_list(self):
        """
        Return a list of tile id's that exist in this map.
        """
        return [int(fn.split('/')[-1].replace('.json', '')) for fn in glob.glob(os.path.join(self.map_dir, '*.json'))]

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
