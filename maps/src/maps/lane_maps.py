from maps import map_types
from maps.utils import geojson_utils
from maps.utils import tile_utils
from maps.utils import translator
from maps.tiled_map import JsonTiledMap
from maps.geojson_tiled_map import TileDict

LANE_MAP_TILE_LEVEL = 14

class ConvertedLaneMap(JsonTiledMap):

    def __init__(self, lane_tile_dir, cache_tiles=True, load_tiles=True, fix_dot=True):
        """
        :param road_tile_dir: The directory where the road graph tiles are stored / will be stored
        :param tiled_map: optional, a tiled map to generate the road tiles from. If none is provided,
            generate calls will return emtpy dictionaries
        """
        super(ConvertedLaneMap, self).__init__(lane_tile_dir,
                                               LANE_MAP_TILE_LEVEL,
                                               cache_tiles=cache_tiles,
                                               load_tiles=load_tiles)

        self.fix_dot = fix_dot

    # --------------------------------------
    # Disk Operations
    # --------------------------------------

    def load_tile(self, tile_id):
        if not self.tile_exists(tile_id):
            return None

        # load the here maps json tile
        raw_tile = super(ConvertedLaneMap, self).load_tile(tile_id)

        # transloate tile to geojson lane maps
        tile = translator.convert_tile_to_geojson(raw_tile, self.tile_level, self.fix_dot)

        if tile is not None:
            tile = TileDict(tile)
        return tile

    def save_tile(self, tile_id, tile):
        """
        Save a tile to disk in here maps format.

        :param tile_id: id of the tile to fetch
        :param tile: a tile dict in here maps format
        """
        if tile is None:
            return

        # translate tile to here maps json
        raw_tile = translator.convert_geojson_to_tile(tile)

        # save it
        super(ConvertedLaneMap, self).save_tile(tile_id, raw_tile)
