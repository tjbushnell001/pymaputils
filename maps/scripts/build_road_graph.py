#! /usr/bin/env python
"""
Construct a road graph using a tiled lane map as a reference source. First generate the road tiles to give coverage
over the lane map, then relink the road tiles.
"""
import os
import rospkg
import tempfile
import shutil

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
import maps.road_graph
from maps.utils import tile_linker


if __name__ == '__main__':
    map_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')

    # read in linked raw here lane tiles into geo json
    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), fix_dot=True, cache_tiles=False)

    # Build unlinked road tiles (to a temp directory)
    unlinked_road_temp_dir = tempfile.mkdtemp()
    unlinked_road_graph = GeoJsonTiledMapLayer(unlinked_road_temp_dir,
                                               maps.road_graph.ROAD_GRAPH_TILE_LEVEL)

    # Then we link final road graph tiles
    road_graph = GeoJsonTiledMapLayer(os.path.join(map_dir, 'road_tiles'),
                                      maps.road_graph.ROAD_GRAPH_TILE_LEVEL,
                                      cache_tiles=False)

    # Remove existing tiles because we're rebuilding them
    road_graph.remove_tiles()

    road_tile_ids = {maps.road_graph.lane_to_road_tile_id(i) for i in lane_map.get_tile_list()}
    # Build unlinked road tiles
    for road_tile_id in road_tile_ids:
        maps.road_graph.generate_road_tile(road_tile_id, unlinked_road_graph, lane_map)

    # Relink road tiles
    for road_tile_id in road_tile_ids:
        tile_linker.relink_road_tile(road_tile_id, road_graph, unlinked_road_graph)

    # cleanup temp files
    shutil.rmtree(unlinked_road_temp_dir, True)
