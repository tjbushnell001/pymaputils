#! /usr/bin/env python
import os
import glob
import maps.routing
import sys

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.utils import emblog
from maps.utils import routing_utils

# ----------------------------------------------------------------------------------------------------------------
# Because this test is not running in a ROS environment, we can't rely on our ros params or package locations
# for the map data.
# TODO: Make sure to keep these file locations up to date!
#
# We currently do not run this test in Jenkins at the moment as the linter runs fast enough as is
# ----------------------------------------------------------------------------------------------------------------

BRAIN_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../..")
MAP_DIR = os.path.join(BRAIN_ROOT, "perception/lane_map_server/maps/tiled_maps/usa")
MAP_READER_DIR = os.path.join(BRAIN_ROOT, "perception/map_reader/maps")
ROUTES_DIR = os.path.join(BRAIN_ROOT, "brain/params/routes")


def main():
    route_ids = set()
    for fn in glob.glob(os.path.join(ROUTES_DIR, '*.yaml')):
        route_id = os.path.splitext(os.path.basename(fn))[0]
        route_ids.add(route_id)

    road_graph = GeoJsonTiledMapLayer(os.path.join(MAP_DIR, 'road_tiles'), tile_level=ROAD_GRAPH_TILE_LEVEL)

    failed_routes = set()
    for route_id in route_ids:
        print
        print "*****************************************"
        print "Generating route for [{}]".format(route_id)

        all_waypoints = routing_utils.load_waypoints_from_map_reader(MAP_READER_DIR, route_id)
        routes = maps.routing.find_route(road_graph, all_waypoints, routing_utils.Capabilities())

        if routes is None:
            failed_routes.add(route_id)
            emblog.error("ROUTE FAILURE: Failed to generate route for [{}]".format(route_id))
        else:
            emblog.debug("[{}] Passed.".format(route_id))

    if len(failed_routes):
        emblog.error("\n"
                     "**************************************\n"
                     "* Route Generation FAILED!\n"
                     "**************************************\n")
        sys.exit(1)
    else:
        emblog.debug("\nRoute Gen PASSED.\n")


if __name__ == '__main__':
    main()
