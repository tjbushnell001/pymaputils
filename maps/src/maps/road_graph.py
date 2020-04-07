import geojson
import itertools
import utm

from maps.utils import ref_utils
from maps.utils import geojson_utils
from maps.utils import tile_utils
from maps.feature_dict import FeatureDict

from maps.lane_maps import LANE_MAP_TILE_LEVEL
ROAD_GRAPH_TILE_LEVEL = 10


def generate_road_tile(road_tile_id, road_graph, lane_map, save_tiles=True):
    """
    Generate a road tile given a road tile id and the associated lane data in lane map. Store the new
    tiles in road_graph. If save_tiles, tiles will be written to disk
    """
    # If our lane map has not been dot corrected, road generation will fail
    assert lane_map.fix_dot

    sub_tile_ids = tile_utils.sub_tile_ids(road_tile_id, road_graph.tile_level, lane_map.tile_level)

    sub_tiles = []
    for tid in sub_tile_ids:
        sub_tile = lane_map.get_tile(tid)
        if sub_tile is not None:
            sub_tiles.append(sub_tile)

    if len(sub_tiles) == 0:
        # no road tiles
        return None

    road_tile = build_road_tile(road_graph, road_tile_id, sub_tiles)
    if road_tile is not None:
        road_tile = FeatureDict(road_tile)

    road_graph.add_tile(road_tile_id, road_tile)
    if save_tiles:
        road_graph.save_tile(road_tile_id, road_tile)

    return road_tile


def lane_to_road_tile_id(lane_map_tile_id):
    """ Convert a lane tile id to it's parent road graph tile id. """
    return tile_utils.super_tile_id(lane_map_tile_id, LANE_MAP_TILE_LEVEL, ROAD_GRAPH_TILE_LEVEL)


# ----------------------------------------------------------
# Helper Methods
# ----------------------------------------------------------

def traverse_lane_groups(lane_group, all_lane_groups, all_connectors, reverse=False):
    """
    Starting at lane_group, create a chain of lane groups, traversing and appending until you hit some stopping
    conditions. Return this list as a path. This will construct a road segment lane group list.
    """
    path = []

    property_keys = ['direction_of_travel', 'is_ramp', 'functional_class', 'is_controlled_access']
    path_properties = [lane_group.properties[k] for k in  property_keys]
    while lane_group is not None:
        lg_properties = [lane_group.properties[k] for k in property_keys]
        if path_properties != lg_properties:
            # only traverse lane groups with the same propertiess
            break

        path.append(lane_group)

        connector_ref = lane_group.properties['start_connector_ref' if reverse else 'end_connector_ref']
        connector = all_connectors.get(connector_ref)
        if connector is None:
            # can't find connector
            break

        if len(connector.properties['outflow_refs']) != 1 or len(connector.properties['inflow_refs']) != 1:
            break

        lg_ref = connector.properties['inflow_refs' if reverse else 'outflow_refs'][0]
        lane_group = all_lane_groups.get(lg_ref)

    return path


def retile_ref(ref):
    """ Create a road ref from an associated lane map ref. """
    road_tile_id = lane_to_road_tile_id(ref['tile_id'])

    ref_type = ref['type']
    if ref_type == 'lane_group_ref':
        new_ref = ref_utils.create_road_segment_ref(road_tile_id, ref['id'])
    elif ref_type == 'connector_ref':
        new_ref = ref_utils.create_road_connector_ref(road_tile_id, ref['id'])
    else:
        assert False, 'Invalid ref type: %s' % ref_type

    return new_ref


def build_road_tile(road_graph, road_tile_id, sub_tiles):
    """ Build route tile from map_tile """
    used_lane_groups = set()

    min_lat, min_lng, max_lat, max_lng = tile_utils.tile_bounds(road_tile_id, road_graph.tile_level)
    center_lat, center_lng = (min_lat + max_lat) / 2, (min_lng + max_lng) / 2
    utm_zone, utm_lat_band = utm.from_latlon(center_lat, center_lng)[2:]

    all_lane_groups = {}
    all_connectors = {}
    for sub_tile in sub_tiles:
        all_lane_groups.update(sub_tile.get_features('lane_group'))
        all_connectors.update(sub_tile.get_features('connector'))

    features = []
    skipped_junctions = set()
    for lg_id, lg in all_lane_groups.iteritems():
        if lg_id in used_lane_groups:
            continue

        dot = lg.properties['direction_of_travel']
        invalid = dot != 'FORWARD'

        path = traverse_lane_groups(lg, all_lane_groups, all_connectors, reverse=True)
        path.reverse()
        path.pop()
        path.extend(traverse_lane_groups(lg, all_lane_groups, all_connectors))

        path_refs = [l.ref for l in path]

        # don't reprocess any of these lane groups
        used_lane_groups.update(path_refs)

        first_lg = path[0]
        last_lg = path[-1]

        start_connector_ref = first_lg.properties['start_connector_ref']
        end_connector_ref = last_lg.properties['end_connector_ref']

        left_boundaries = itertools.chain(*[l.properties['left_boundary'] for l in path])
        right_boundaries = itertools.chain(*[l.properties['right_boundary'] for l in path])

        left_boundary = geojson_utils.downsample_line(left_boundaries, utm_zone, utm_lat_band)
        right_boundary = geojson_utils.downsample_line(right_boundaries, utm_zone, utm_lat_band)

        length = sum((l.properties['length'] for l in path))

        for lg in path[:-1]:
            connector_ref = lg.properties['end_connector_ref']
            skipped_junctions.add(connector_ref)

        road_segment_ref = retile_ref(path_refs[0])
        # assert road_segment_id[:2] == road_tile_id, (road_segment_id, road_tile_id, first_lg.id)

        boundary = geojson_utils.boundaries_to_poly(left_boundary, right_boundary)

        road_segment = geojson_utils.create_feature(
            'road_segment',
            road_segment_ref,
            boundary,
            lane_group_refs=path_refs,
            start_connector_ref=retile_ref(start_connector_ref),
            end_connector_ref=retile_ref(end_connector_ref),
            left_boundary=left_boundary,
            right_boundary=right_boundary,
            length=length,
            direction_of_travel = dot,
            is_ramp = first_lg.properties['is_ramp'],
            functional_class = first_lg.properties['functional_class'],
            is_controlled_access = first_lg.properties['is_controlled_access'],
            invalid=invalid)

        features.append(road_segment)

    for connector_ref, connector in all_connectors.iteritems():
        if connector.ref in skipped_junctions:
            continue

        road_connector_ref = retile_ref(connector.ref)

        road_connector = geojson_utils.create_feature(
            'road_connector',
            road_connector_ref,
            geojson.LineString(connector.geometry.coordinates),
            inflow_refs=[],
            outflow_refs=[])

        # assert road_junction_id[:2] == road_tile_id, (road_junction_id, road_tile_id)

        features.append(road_connector)

    road_tile = geojson_utils.create_feature_collection(
        'road_tile', road_tile_id, features,
        tile_level=road_graph.tile_level,
        utm_zone=utm_zone,
        utm_lat_band=utm_lat_band,
        min_lat=min_lat,
        min_lng=min_lng,
        max_lat=max_lat,
        max_lng=max_lng
    )
    return road_tile
