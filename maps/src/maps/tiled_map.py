import glob
import json
import os

from maps.utils import tile_utils

class TiledMap(object):
    """
    Class that represents a tiled map and it's directory structure. No longer loads a single monolithic map. Instead
    this edits individual map files.
    """

    def __init__(self, map_dir, tile_level, cache_tiles=True, load_tiles=True):
        """
        :param map_dir: The root directory of this tiled map (usually lives one level below tiled_maps)
        :param caching: Whether or not to cache tiles
        :param tile_level: the here_maps tile level. This basically defines the resolution of the tiles (larger number
            means smaller tiles).
        """
        self.map_dir = map_dir
        self.tile_level = tile_level

        self.cache_tiles = cache_tiles
        self.load_tiles = load_tiles

        # tile cache
        self.tiles = {}

    def get_tile(self, tile_id):
        tile = None
        if self.cache_tiles and tile_id in self.tiles:
            tile = self.tiles[tile_id]

        elif self.load_tiles:
            tile = self.load_tile(tile_id)
            if self.cache_tiles:
                self.tiles[tile_id] = tile

        return tile

    def add_tile(self, tile_id, tile):
        if self.cache_tiles:
            self.tiles[tile_id] = tile

    def clear_cache(self):
        self.cache = {}

    # ---------------------------------------
    # Disk operations to access map files
    # ---------------------------------------
    def load_tile(self, tile_id):
        """
        Load a here map tile and return it.

        :param tile_id: int id of the tile (used to name / fetch json tile files)
        """
        raise NotImplementedError()

    def save_tile(self, tile_id, tile):
        """
        Save a tile to disk in here maps format.

        :param tile_id: id of the tile to fetch
        :param tile: a tile dict in here maps format
        """
        raise NotImplementedError()

    def get_tile_list(self):
        """
        Return a list of tile id's that exist in this map.
        """
        raise NotImplementedError()


    # ---------------------------------------
    # Other API Methods
    # ---------------------------------------

    def remove_tiles(self):
        """ Remove all tiles on disk. """
        # clear cache
        self.tiles = {}

    def tile_exists(self, tile_id):
        """
        Return true if the tile exists

        :param tile_id: The id of the tile to check for
        :return: boolean
        """
        raise NotImplementedError()

    def yield_adjacent_tiles(self, tile_id):
        """
        Yield a tile in the adjacency set of tile with id tile_id.

        :param tile_id: id of tile around which to search
        """
        for adjacent_tile_id in tile_utils.adjacent_tile_ids(tile_id, self.tile_level, include_self=True):
            tile = self.get_tile(adjacent_tile_id)
            if tile is not None:
                yield tile

class JsonTiledMap(TiledMap):
    def __init__(self, map_dir, tile_level, cache_tiles=True, load_tiles=True):
        super(JsonTiledMap, self).__init__(map_dir, tile_level, cache_tiles=cache_tiles, load_tiles=load_tiles)

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
            print "Loaded tile: {}".format(fn)
            return tile

    def save_tile(self, tile_id, tile):
        """
        Save a tile to disk in here maps format.

        :param tile_id: id of the tile to fetch
        :param tile: a tile dict in here maps format
        """
        if tile is None:
            return

        fn = self.get_tile_filename(tile_id)
        with open(fn, 'w') as f:
            json.dump(tile, f, indent=4, sort_keys=True)
            print "Saved tile: {}".format(fn)

    def get_tile_list(self):
        """
        Return a list of tile id's that exist in this map.
        """
        return [int(fn.split('/')[-1][:-5]) for fn in glob.glob(os.path.join(self.map_dir, '*.json'))]

    # ---------------------------------------
    # Other API Methods
    # ---------------------------------------

    def remove_tiles(self):
        """ Remove all tiles on disk. """
        for fn in glob.glob(os.path.join(self.map_dir, '*.json')):
            os.unlink(fn)

        # make sure to clear the cache
        super(JsonTiledMap, self).remove_tiles()

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
