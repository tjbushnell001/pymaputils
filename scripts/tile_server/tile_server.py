#! /usr/bin/env python
import argparse
import flask
import flask_cors
import geojson
import glob
import maps
import maps.road_graph
import os
import re

from copy import copy
from flask_cors import cross_origin
from maps.feature_dict import FeatureDict
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.map_layers import MapLayers
from maps.map_types import MapType
from maps.utils import geojson_utils
from maps.utils import tile_linker
from maps.utils import tile_utils
from maps.utils import ref_utils

import shapely.geometry


def parse_args(args=None):
    parser = argparse.ArgumentParser(description="Mappy tile server")
    parser.add_argument('--map_dir', type=str, required=True,
                        help="Location of map data (root of tiled_maps) on the local machine")
    return parser.parse_args(args=args)


# ------------------------------------
# Constants
# ------------------------------------

PORT = 8990
VALID_ROUTE_NAME = re.compile('^[a-zA-Z0-9_-]+$')

app = flask.Flask(__name__)
app.config['CORS_HEADERS'] = 'Content-Type'
cors = flask_cors.CORS(app, resources={r"/*": {"origins": "maps.embarktrucks.com"}})

lane_map = None
lidar_map_layer = None
dot_corrected_lane_map = None
road_graph = None
map_reader_dir = None


# ------------------------------------
# Helper Methods
# ------------------------------------

def get_tile_boundary_feature_collection(tile_ids, feature_type, tile_level):
    features = []
    for tile_id in tile_ids:
        bbox = maps.utils.tile_utils.tile_bounds(tile_id, tile_level)
        utm_zone, utm_lat_band = maps.utils.tile_utils.tile_utm_zone(tile_id, tile_level)
        f = geojson.Feature(geometry=geojson_utils.bbox_to_poly(*bbox),
                            id=tile_id,
                            feature_type=feature_type,
                            properties={
                                "tile_level": tile_level,
                                "utm_zone": utm_zone,
                                "utm_lat_band": utm_lat_band})

        features.append(f)

    fc = geojson.FeatureCollection(features)
    return flask.jsonify(fc)


# ------------------------------------
# REST api
# ------------------------------------

@app.route("/tiles", methods=['GET'])
@app.route("/tiles/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_lane_tiles():
    """ Return a feature collection of tile boundaries of all lane tiles. """
    tile_ids = lane_map.get_tile_list()
    if len(tile_ids) == 0:
        print 'No lane tiles found! Did you run "git submodule update --init" to populate tiled_maps?'
        return flask.Response(response="No tiles founds.", status=204)
    return get_tile_boundary_feature_collection(tile_ids, 'tile', lane_map.tile_level)


@app.route("/tiles/<int:tile_id>", methods=['GET'])
@app.route("/tiles/<int:tile_id>/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_lane_tile(tile_id):
    """
    Load the tile associated with tile_id from disk and return it as a geojson object.
    :param tile_id: the id of the tile being loaded
    :return: the geojson lane tile data as a json object
    """
    tile_id = int(tile_id)
    tile = lane_map.get_tile(tile_id)
    if not tile:
        flask.abort(404)
        return

    lidar_tile = lidar_map_layer.get_tile(tile_id)

    if lidar_tile:
        collection_copy = copy(tile.collection)
        collection_copy.features = copy(tile.collection.features) + lidar_tile.collection.features

        return flask.jsonify(collection_copy)

    return flask.jsonify(tile.collection)

@app.route("/tiles/find", methods=['GET'])
@app.route("/tiles/find/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_lane_feature():
    """
    Find coordinates for the specified feature ref.  Ref is encoded as url args.
    (e.g. "/tiles/find/?type=lane_ref&tile_id=309134521&lane_group_id=2083391152&id=2")
    :return: A feature with the coordinates
    """
    # create a ref from url arguments
    ref = dict(flask.request.args)
    for k,v in ref.items():
        v = v[0]
        if k != 'type':
            v = int(v)
        ref[k] = v
    ref = ref_utils.hashify(ref)

    # find the feature
    feature = lane_map.get_feature(ref)

    if feature is None:
        flask.abort(404)
        return

    # create a result with a representative point
    geom = shapely.geometry.shape(feature.geometry)
    point = geom.representative_point()

    f = geojson.Feature(geometry=point,
                        properties={
                            'query' : ref,
                        })

    return flask.jsonify(f)


@app.route("/road_tiles", methods=['GET'])
@app.route("/road_tiles/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_road_tiles():
    """ Return a feature collection of road tile boundaries of all road tiles. """
    tile_ids = road_graph.get_tile_list()
    if len(tile_ids) == 0:
        print 'No road tiles found! Did you run "git submodule update --init" to populate tiled_maps?'
        return flask.Response(response="No tiles founds.", status=204)
    return get_tile_boundary_feature_collection(tile_ids, 'road_tile', road_graph.tile_level)


@app.route("/road_tiles/<int:tile_id>", methods=['GET'])
@app.route("/road_tiles/<int:tile_id>/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_road_tile(tile_id):
    """
    Load the road tile associated with tile_id from disk and return it as a geojson object.
    :param tile_id: the id of the road tile being loaded
    :return: the geojson road tile data as a json object
    """
    tile = road_graph.get_tile(int(tile_id))
    if tile is None:
        # not found
        flask.abort(404)
        return

    return flask.jsonify(tile.collection)


@app.route("/routes", methods=['GET'])
@app.route("/routes/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_map_reader_routes():
    """ Get a list of map reader routes available on the truck. """
    map_files = glob.glob(os.path.join(map_reader_dir, '*.json'))

    map_names = [os.path.splitext(os.path.basename(f))[0] for f in map_files]
    return flask.jsonify(map_names)


@app.route("/routes/<route_name>", methods=['GET'])
@app.route("/routes/<route_name>/", methods=['GET'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def get_map_reader_route(route_name):
    """ Load a geojson route and return it """
    if VALID_ROUTE_NAME.match(route_name) is None:
        # invalid route name
        flask.abort(400)
        return

    fn = os.path.join(map_reader_dir, route_name + '.json')
    if not os.path.exists(fn):
        # route doesn't exist
        flask.abort(404)
        return

    with open(fn, 'r') as f:
        content = geojson.load(f)
    return flask.jsonify(content)


@app.route("/routes/<route_name>", methods=['PUT'])
@app.route("/routes/<route_name>/", methods=['PUT'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def save_map_reader_route(route_name):
    """ Save a geojson route file """
    if VALID_ROUTE_NAME.match(route_name) is None:
        # invalid route name
        flask.abort(400)
        return

    fn = os.path.join(map_reader_dir, route_name + '.json')
    if not os.path.exists(fn):
        # route doesn't exist
        flask.abort(404)
        return

    route = geojson.loads(flask.request.data)
    geojson_utils.write_geojson_object(route_name, map_reader_dir, route)
    # geojson.dump(route, open(fn, 'w'), sort_keys=True, separators=(',', ':'), indent=0)

    return flask.Response(status=200)


def rebuild_road_tiles(lane_tile_id):
    """
    Rebuild all road tiles connected to this lane_tile_id in order to propagate changes to the
    road graph.
    """
    # re-generate all required unlinked road tiles
    road_tile_id = maps.road_graph.lane_to_road_tile_id(lane_tile_id)

    # relink all lane tiles in the immediate area of this change
    for adjacent_tile_id in tile_utils.adjacent_tile_ids(lane_tile_id, lane_map.tile_level, include_self=True):
        tile_linker.relink_lane_tile(adjacent_tile_id, lane_map, lane_map)

    # rebuild affected road tiles

    # we only need a few unlinked road tiles, so just load them in memory
    unlinked_road_graph = GeoJsonTiledMapLayer(None, road_graph.tile_level, load_tiles=False)

    # for speed, copy the existing *linked* adjacent road tiles since they haven't changed
    for adjacent_tile_id in tile_utils.adjacent_tile_ids(road_tile_id, road_graph.tile_level, include_self=False):
        tile = road_graph.get_tile(adjacent_tile_id)
        unlinked_road_graph.add_tile(adjacent_tile_id, tile)

    # but fully regenerate the affected road tile
    maps.road_graph.generate_road_tile(road_tile_id, unlinked_road_graph, dot_corrected_lane_map, save_tiles=False)

    # relink affected road tile
    tile_linker.relink_road_tile(road_tile_id, road_graph, unlinked_road_graph)


@app.route("/tiles/<int:tile_id>", methods=['PUT'])
@app.route("/tiles/<int:tile_id>/", methods=['PUT'])
@cross_origin(origin='localhost', headers=['Content- Type', 'Authorization'])
def update_tile(tile_id):
    """
    Replace the tile data associated with tile id with the data from the request body.
    For right now, tile updates
    purely replace the full tile.
    TODO: implement partial patching logic
    :param tile_id: the id of the lane tile being replaced
    :return: a flask response object
    """
    print "Saving tile [{}], please wait...".format(tile_id)

    features = geojson.loads(flask.request.data)

    # Filter out lidar data in tile
    features = [feature for feature in features if feature['type'] != 'lidar_segment']
    tile = FeatureDict(features)

    lane_map.save_tile(tile_id, tile)

    print "Rebuilding road tiles, please wait..."
    rebuild_road_tiles(tile_id)
    print "Done rebuilding."

    return flask.Response(status=200)


def main():
    # Using globals because flask is not meant to be a robust service and doesn't allow for smarter
    # design
    # TODO(christian): move on from flask.
    global map_reader_dir  # pylint: disable=global-statement
    global lidar_map_layer  # pylint: disable=global-statement
    global lane_map  # pylint: disable=global-statement
    global road_graph  # pylint: disable=global-statement
    global dot_corrected_lane_map  # pylint: disable=global-statement

    args = parse_args()

    if not os.path.isdir(args.map_dir):
        exit("Directory [{}] does not exist! Please provide correct directory to --map_dir".format(args.map_dir))

    map_dir = os.path.join(args.map_dir, "usa")
    map_reader_dir = os.path.join(map_dir, "map_reader")

    map_layers = MapLayers(map_dir=map_dir, map_reader_dir=map_reader_dir)

    lidar_map_layer = map_layers.get_layer(MapType.LIDAR_LINE)
    lane_map = map_layers.get_layer(MapType.LANE, cache_tiles=False, fix_dot=False)
    road_graph = map_layers.get_layer(MapType.ROAD, cache_tiles=False)

    # TODO: This is a hack until we clean up the map layers api. Users should never have to
    #  manually instantiate a map layer class but the multiple versions of the lane map layer
    #  with different configurations is a unique case that requires a more robust api
    dot_corrected_lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'),
                                                   cache_tiles=False, fix_dot=True)

    app.run(debug=False, port=PORT, threaded=True)


if __name__ == '__main__':
    main()
