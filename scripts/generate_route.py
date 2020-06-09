#! /usr/bin/env python
"""
Takes a set of route_ids (i.e. the names of our route files) and converts them from simple map annotations into full
route layers.
"""
import argparse
import os

from maps.utils import geojson_utils, routing_utils
import maps.routing
import maps.map_layers
from maps.map_types import MapType

def parse_args():
    parser = argparse.ArgumentParser("Generate a route given a route id and save to a file.")

    parser.add_argument('maps_dir', help='directory containing maps')
    parser.add_argument('route_ids', nargs='+', help='route id to be generated')
    parser.add_argument('--route_dir', default="/tmp", help='Output directory for routes')
    return parser.parse_args()


def main():
    args = parse_args()

    map_reader_dir = os.path.join(args.maps_dir, 'map_reader')
    layers = maps.map_layers.MapLayers(map_dir=args.maps_dir,
                                       map_reader_dir=map_reader_dir)

    road_graph = layers.get_layer(MapType.ROAD, cache_tiles=True)

    for route_id in args.route_ids:
        route_map = layers.get_layer(MapType.MAP_READER, layer_name=route_id)

        # load waypoints for this route
        all_waypoints = routing_utils.create_waypoints_from_map_reader(route_map, route_id)

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

        # TODO: convert this into a FeatureDict object
        geojson_utils.write_geojson_object(route_id, args.route_dir,
                                           geojson_utils.create_feature_collection('route', route_id, features))
        print 'Wrote', os.path.join(args.route_dir, '{}.json'.format(route_id))


if __name__ == '__main__':
    main()
