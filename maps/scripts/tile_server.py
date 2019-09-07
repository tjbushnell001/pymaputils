#! /usr/bin/env python
import flask
import flask_cors
import geojson
import glob
import maps
import os
import re
import time

from flask_cors import cross_origin
import maps.road_graph
from maps.geojson_tiled_map import GeoJsonTiledMapLayer
from maps.lane_maps import ConvertedLaneMapLayer
from maps.utils import geojson_utils
from maps.utils import tile_linker
from maps.utils import tile_utils
from maps.feature_layer import FeatureLayer


# ------------------------------------
# Constants
# ------------------------------------

PORT = 8990
VALID_ROUTE_NAME = re.compile('^[a-zA-Z0-9_-]+$')

app = flask.Flask(__name__)
app.config['CORS_HEADERS'] = 'Content-Type'
cors = flask_cors.CORS(app, resources={r"/foo": {"origins": "http://localhost:port"}})

lane_map = None
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
    tile = lane_map.get_tile(int(tile_id))
    if not tile:
        flask.abort(404)
        return
    return flask.jsonify(tile.collection)


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


def rebuild_road_tiles(lane_tile_id):
    """ Rebuild all road tiles connected to this lane_tile_id in order to propagate changes to the road graph. """
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
    Replace the tile data associated with tile id with the data from the request body. For right now, tile updates
    purely replace the full tile.
    TODO: implement partial patching logic

    :param tile_id: the id of the lane tile being replaced
    :return: a flask response object
    """
    print "Saving tile [{}], please wait...".format(tile_id)

    tile = FeatureDict(geojson.loads(flask.request.data))
    lane_map.save_tile(tile_id, tile)

    print "Rebuilding road tiles, please wait..."
    rebuild_road_tiles(tile_id)
    print "Done rebuilding."

    return flask.Response(status=200)


if __name__ == '__main__':
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
    map_dir = os.path.join(base_dir, "perception/lane_map_server/maps/tiled_maps/usa")
    map_reader_dir = os.path.join(base_dir, "perception/map_reader/maps")

    lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), cache_tiles=False, fix_dot=False)
    dot_corrected_lane_map = ConvertedLaneMapLayer(os.path.join(map_dir, 'tiles'), cache_tiles=False, fix_dot=True)
    road_graph = GeoJsonTiledMapLayer(os.path.join(map_dir, 'road_tiles'), maps.road_graph.ROAD_GRAPH_TILE_LEVEL,
                                      cache_tiles=False)

    app.run(debug=False, port=PORT, threaded=True)
