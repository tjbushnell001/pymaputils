from maps.utils import translator
from maps.geojson_maps import TileDict
from maps.tiled_map_layer import JsonTiledMapLayer

LANE_MAP_TILE_LEVEL = 14


class ConvertedLaneMapLayer(JsonTiledMapLayer):

    def __init__(self, lane_tile_dir, cache_tiles=True, load_tiles=True, fix_dot=True):
        """
        :param lane_tile_dir: The directory where the lane tiles are stored / will be stored
        :param cache_tiles: Whether to cache a copy of tiles in memory
        :param load_tiles: Whether to load tiles from disk if none are cached
        :param fix_dot: Whether to fix the direction of travel on loaded tiles
        """
        super(ConvertedLaneMapLayer, self).__init__(lane_tile_dir, LANE_MAP_TILE_LEVEL, cache_tiles=cache_tiles,
                                                    load_tiles=load_tiles)

        self.fix_dot = fix_dot

    # --------------------------------------
    # Disk Operations
    # --------------------------------------

    def load_tile(self, tile_id):
        if not self.tile_exists(tile_id):
            return None

        # load the here maps json tile
        raw_tile = super(ConvertedLaneMapLayer, self).load_tile(tile_id)

        # translate tile to geojson lane maps
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
        super(ConvertedLaneMapLayer, self).save_tile(tile_id, raw_tile)
