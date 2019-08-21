#! /usr/bin/env python
"""
Takes a set of route_ids (i.e. the names of our route files) and converts them from simple map annotations into full
route layers.
"""
import argparse
import os
import rospkg

from maps.geojson_maps import GeoJsonTiledMapLayer
from maps.utils import geojson_utils, routing_utils
import maps.routing
import maps.road_graph

parser = argparse.ArgumentParser("Generate a route given a route id and save to a file.")
parser.add_argument('route_ids', nargs='+', help='route id to be generated')
parser.add_argument('--route_dir', default="/tmp", help='Output directory for routes')

if __name__ == '__main__':
    args = parser.parse_args()

    maps_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')
    map_reader_dir = os.path.join(rospkg.RosPack().get_path('map_reader'), 'maps')

    road_graph = GeoJsonTiledMapLayer(os.path.join(maps_dir, 'road_tiles'),
                                      maps.road_graph.ROAD_GRAPH_TILE_LEVEL)

    for route_id in args.route_ids:
        # load waypoints from map reader
        all_waypoints = routing_utils.load_waypoints_from_map_reader(map_reader_dir, route_id)

        # find the route
        routes = maps.routing.find_route(road_graph, all_waypoints,
                                         routing_utils.Capabilities())

        if routes is None:
            print "Failed to generate route for route id {}".format(route_id)
            continue

        features = []

        for route_road_refs, route_waypoint_ids in routes:
            route_feature = routing_utils.create_route_feature(road_graph, route_road_refs, route_waypoint_ids)

            features.append(route_feature)
        features.extend(all_waypoints)

        # TODO: convert this into a TiledMapLayer object
        geojson_utils.write_geojson_tile(route_id, args.route_dir,
                                         geojson_utils.create_feature_collection('route', route_id, features))
        print 'Wrote', os.path.join(args.route_dir, '{}.json'.format(route_id))
