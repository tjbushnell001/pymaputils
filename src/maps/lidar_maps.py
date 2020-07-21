import glob
import os
from maps.map_types import MapType
from maps import feature_dict
from maps.tiled_map_layer import JsonTiledMapLayer
from lane_maps import LANE_MAP_TILE_LEVEL
from lidar_lane_generation import lidar_gen_utils

# 
class LidarLineLayer(JsonTiledMapLayer):
    def __init__(self, map_dir, cache_tiles=True, load_tiles=True):
        super(LidarLineLayer, self).__init__(map_dir, LANE_MAP_TILE_LEVEL, cache_tiles=cache_tiles,
                                                    load_tiles=load_tiles, layer_type=MapType.LIDAR_LINE,
                                                    json_separators=(', ', ': '))
    def load_tile(self, tile_id):
        if not self.tile_exists(tile_id):
            return None


        fn = self.get_tile_filename(tile_id)

        # load the here maps json tile
        #raw_tile = super(LidarLineLayer, self).load_tile(tile_id)

        # translate tile to geojson lane maps
        #tile = translator.convert_tile_to_geojson(raw_tile, self.tile_level, self.fix_dot)

        fd = feature_dict.load_from_file(fn)
            
        return lidar_gen_utils.geojson_to_lane_segments(fd)
    
    def save_tile(self, tile_id, tile):
        raw_tile = lidar_gen_utils.lat_lng_lane_segments_to_geojson(tile, tile_id)
        super(LidarLineLayer, self).save_tile(tile_id, raw_tile)

        self.add_tile(tile_id, tile)
