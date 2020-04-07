#! /usr/bin/env python
""" Simple script to run relinking over all here map lane tiles. """
import argparse
import os
import rospkg

from maps.lane_maps import ConvertedLaneMapLayer
from maps.utils import tile_linker

parser = argparse.ArgumentParser("Build road graph tiles using tiled map.")
parser.add_argument("--tile_dir", default=None, help='where to find the tiled maps')

if __name__ == '__main__':
    args = parser.parse_args()

    map_dir = args.tile_dir
    if map_dir is None:
        map_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')

    # read in raw here lane tiles into geo json
    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), fix_dot=False)

    for tile_id in lane_map.get_tile_list():
        tile_linker.relink_lane_tile(tile_id, lane_map, lane_map)
