"""
Converts GeoJSON formatted map features back to raw here maps json format for backwards compatibility
with the old map editor.
"""
import utm
import geojson
import numpy as np

from maps import map_types
from maps.utils import geojson_utils
from maps.utils import tile_utils

# ---------------------------------------
# Constants
# ---------------------------------------

# These lists denote a set of properties that are copied directly between the translation with no extra logic
LANE_FEATURE_LIST = [
    'altitude_pts',
    'direction_of_travel',
    'from_split',
    'is_emergency_lane',
    'is_transitioning',
    'lane_transition_type',
    'lane_type',
    'merged',
    'merging',
    'upcoming_split',
]

LANE_BOUNDARY_FEATURE_LIST = [
    'color',
    'is_only_emergency_boundary',
    'marking_width',
    'material',
    'style',
]

LANE_GROUP_FEATURE_LIST = [
    'intersecting_tiles',
    'left_boundary',
    'right_boundary',
    'length',
    'functional_class',
    'route_type',
    'is_accessible_by_pedestrians',
    'is_built_up_area_road',
    'is_controlled_access',
    'is_limited_access',
    'is_multiply_digitized',
    'is_ramp',
    'is_tunnel',
    'is_urban',
    'is_within_interchange',
]

VALID_LANE_TYPES = {
    'UNKNOWN', 'REGULAR', 'HIGH_OCCUPANCY_VEHICLE', 'DRIVABLE_SHOULDER',
    'BICYCLE', 'PARKING', 'REVERSIBLE', 'EXPRESS', 'ACCELERATION',
    'DECELERATION', 'AUXILIARY', 'SLOW', 'PASSING', 'REGULATED_ACCESS',
    'TURN', 'CENTRE_TURN', 'TRUCK_PARKING', 'SHOULDER', 'VARIABLE_DRIVING',
    'DRIVABLE_PARKING', 'OTHER'
}

VALID_LANE_TRANSITION_TYPES = {None, 'UNKNOWN', 'MERGE', 'SPLIT'}


# ---------------------------------------
# Maps Translation API
# ---------------------------------------

def convert_geojson_to_tile(geojson_tile):
    """
    Converts a GeoJSON formatted feature group tile into a here maps json formatted tile

    :param geojson_tile: the GeoJSON feature group (a python dict, NOT TileDict)
    :return: json object
    """
    # Associate each junction with it's connector
    connectors = [feature_to_connector(v)
                  for v in geojson_tile.get_features('connector').values()]
    connectors_by_id = {c['id'] : c for c in connectors}

    for feature in geojson_tile.get_features('junction').values():
        conn_id = feature.ref['connector_id']
        if conn_id in connectors_by_id:
            junc = feature_to_junction(feature)
            connectors_by_id[conn_id]['junctions'].append(junc)

    # Associate lanes with their lane groups
    lane_groups = [feature_to_lane_group(v)
                   for v in geojson_tile.get_features('lane_group').values()]
    lane_groups_by_id = {lg['id'] : lg for lg in lane_groups}
    for feature in geojson_tile.get_features('lane').values():
        lg_id = feature.ref['lane_group_id']
        lane = feature_to_lane(feature)
        if lg_id in lane_groups_by_id:
            lane_groups_by_id[lg_id]['lanes'].append(lane)

    # Associate lane boundaries with their lane groups
    for feature in geojson_tile.get_features('lane_boundary').values():
        lg_id = feature.ref['lane_group_id']
        lb = feature_to_lane_boundary(feature)
        if lg_id in lane_groups_by_id:
            lane_groups_by_id[lg_id]['boundaries'].append(lb)

    return {
        'connectors': connectors,
        'id': geojson_tile.tile.id,
        'lane_groups': lane_groups,
        'type': 'tile',
    }


def convert_tile_to_geojson(here_map_tile, tile_level, fix_dot=True):
    """
    Converts a here map formatted tile into a GeoJSON formatted feature group

    :param here_map_tile: the here map tile (a python dict)
    :param tile_level: the here map tile level used to create this tile (usually 14)
    :param fix_dot: if true, the direction of travel will be scrubbed to make sure all lane groups face forward.
    :return: json object
    """
    features = []

    assert here_map_tile['type'] == 'tile'
    tile_id = here_map_tile['id']

    # Map Tile
    min_lat, min_lng, max_lat, max_lng = tile_utils.tile_bounds(tile_id, tile_level)
    center_lat, center_lng = (min_lat + max_lat) / 2, (min_lng + max_lng) / 2
    utm_zone, utm_lat_band = utm.from_latlon(center_lat, center_lng)[2:]

    for raw_lg in here_map_tile['lane_groups']:
        # Lane Groups
        raw_lane_group_id = raw_lg['id']
        lane_group_dot = determine_direction_of_travel(raw_lg)
        reverse_dot = fix_dot and lane_group_dot == 'BACKWARD'

        lane_group = convert_lane_group_to_geojson(raw_lg, tile_id, utm_zone, reverse_dot=reverse_dot)
        lane_segment_refs = lane_group.properties['lane_segment_refs']
        features.append(lane_group)

        # Lanes
        # lanes should be ordered left to right
        lg_lanes = raw_lg['lanes']
        if reverse_dot:
            lg_lanes = reversed(lg_lanes)

        for lane_idx, raw_lane in enumerate(lg_lanes):
            lane_seg = convert_lane_to_geojson(raw_lane, tile_id, raw_lane_group_id, lane_idx,
                                               lane_group.properties['start_connector_ref'],
                                               lane_group.properties['end_connector_ref'],
                                               reverse_dot=reverse_dot)
            lane_segment_refs.append(lane_seg.ref)
            features.append(lane_seg)

        # Boundaries
        for raw_boundary in raw_lg['boundaries']:
            boundary = convert_boundary_to_geojson(raw_boundary, tile_id, raw_lane_group_id, reverse_dot=reverse_dot)
            features.append(boundary)

    # Connectors
    for raw_conn in here_map_tile['connectors']:
        raw_connector_id = raw_conn['id']
        connector = convert_connector_to_geojson(raw_conn, tile_id)
        features.append(connector)

        for raw_junction in raw_conn['junctions']:
            junction = convert_junction_to_geojson(raw_junction, tile_id, raw_connector_id)
            features.append(junction)

    tile = geojson_utils.create_feature_collection(
        'tile', tile_id, features,
        tile_level=tile_level,
        utm_zone=utm_zone,
        utm_lat_band=utm_lat_band,
        min_lat=min_lat,
        min_lng=min_lng,
        max_lat=max_lat,
        max_lng=max_lng
    )
    return tile


# -------------------------------------------
# Feature Translation - GeoJSON to Here Maps
# -------------------------------------------

def feature_to_connector(feature):
    """
    Converts a GeoJSON lane type feature to a lane dictionary in here maps json format.

    :param feature: the GeoJSON feature object
    :return: converted dict object
    """
    assert feature['feature_type'] == 'connector'
    return {
        'id': feature.ref['id'],
        'type': feature['feature_type'],
        'boundary_geometry': feature.geometry.coordinates,
        'connected_lane_groups': feature.properties['connected_lane_groups'],
        'inflow_refs': feature.properties['inflow_refs'],
        'outflow_refs': feature.properties['outflow_refs'],
        'junctions': []  # empty list of junctions to be populated during reassembly
    }


def feature_to_lane(feature):
    """
    Converts a GeoJSON lane type feature to a lane dictionary in here maps json format. The lane group id is
    returned as well so that the lane can be associated with it's appropriate lane group in reassembly.

    :param feature: the GeoJSON feature object
    :return: pair-wise tuple where the first value is the lane group number and the second value is the json
    """
    assert feature['feature_type'] == 'lane'
    obj = {
        'pts': feature.geometry.coordinates,
        'id': feature.ref['id'],
        'end_connector_lane_number': feature.properties['end_junction_ref']['id'],
        'left_boundary_id': feature.properties['left_boundary_ref']['id'],
        'right_boundary_id': feature.properties['right_boundary_ref']['id'],
        'start_connector_lane_number': feature.properties['start_junction_ref']['id'],
        'type': feature['feature_type']
    }
    obj.update({key: feature.properties[key] for key in LANE_FEATURE_LIST})
    return obj


def feature_to_lane_boundary(feature):
    """
    Converts a GeoJSON lane boundary type feature to a lane boundary dictionary in here maps json format.
    The lane group id is returned as well so that the lane boundary can be associated with it's
    appropriate lane group in reassembly.

    :param feature: the GeoJSON feature object
    :return: pair-wise tuple where the first value is the lane group number and the second value is the json
    """
    assert feature['feature_type'] == 'lane_boundary'
    obj = {
        'pts': feature.geometry.coordinates,
        'id': feature.ref['id'],
        'altitude_pts': feature.properties['altitudes'],
        'type': feature['feature_type']
    }
    obj.update({key: feature.properties[key] for key in LANE_BOUNDARY_FEATURE_LIST})
    return obj


def feature_to_lane_group(feature):
    """
    Converts a GeoJSON lane group type feature to a lane group dictionary in here maps json format.

    :param feature: the GeoJSON feature object
    :return: the converted dict object
    """
    assert feature['feature_type'] == 'lane_group'
    obj = {
        'boundaries': [],  # empty list of lane boundaries to be populated during reassembly
        'end_connector_ref': feature.properties['end_connector_ref'],
        'id': feature.ref['id'],
        'lanes': [],  # empty list of lanes to be populated during reassembly
        'start_connector_id': feature.properties['start_connector_ref']['id'],
        'type': feature['feature_type']
    }
    obj.update({key: feature.properties[key] for key in LANE_GROUP_FEATURE_LIST})
    return obj


def feature_to_junction(feature):
    """
    Converts a GeoJSON junction type feature to a junction dictionary in here maps json format.

    :param feature: the GeoJSON feature object
    :return: pair-wise tuple where the first value is the parent connector id and the second value is the json
    """
    assert feature['feature_type'] == 'junction'
    return {
        'id': feature.ref['id'],
        'pt': feature.geometry.coordinates,
        'junction_type': feature.properties['junction_type'],
        'type': feature['feature_type'],
        'inflow_refs': feature.properties['inflow_refs'],
        'outflow_refs': feature.properties['outflow_refs'],
    }


# -------------------------------------------
# Feature Translation - Here Maps to GeoJSON
# -------------------------------------------


def convert_lane_group_to_geojson(raw_lg, tile_id, utm_zone, reverse_dot=False):
    assert raw_lg['type'] == 'lane_group', raw_lg['type']

    start_connector_ref = map_types.create_connector_ref(tile_id, raw_lg['start_connector_id'])
    end_connector_ref = geojson_utils.hashify(raw_lg['end_connector_ref'])

    lane_group_ref = map_types.create_lane_group_ref(tile_id, raw_lg['id'])

    left_boundary = list(raw_lg['left_boundary'])
    right_boundary = list(raw_lg['right_boundary'])

    dot = determine_direction_of_travel(raw_lg)

    if reverse_dot:
        assert dot == 'BACKWARD'

        # swap the connectors
        start_connector_ref, end_connector_ref = end_connector_ref, start_connector_ref

        # reverse the left and right boundary points
        for b in (left_boundary, right_boundary):
            b.reverse()
        # also swap left and right boundaries themselves
        left_boundary, right_boundary = right_boundary, left_boundary

        dot = 'FORWARD'

    # build a boundary geometry out of the left and right lines
    boundary = geojson_utils.boundaries_to_poly(left_boundary, right_boundary)

    functional_class = raw_lg.get('functional_class')
    assert functional_class is None or 0 <= functional_class <= 6, functional_class

    route_type = raw_lg.get('route_type')
    assert route_type is None or 0 <= route_type <= 4, route_type

    # recompute length
    center_length = (boundary_length(left_boundary, utm_zone) +
                     boundary_length(right_boundary, utm_zone)) / 2

    lane_group = geojson_utils.create_feature(
        'lane_group',
        lane_group_ref,
        boundary,
        intersecting_tiles=raw_lg['intersecting_tiles'],
        length=center_length,
        start_connector_ref=start_connector_ref,
        end_connector_ref=end_connector_ref,
        left_boundary=left_boundary,
        right_boundary=right_boundary,
        direction_of_travel=dot,
        functional_class=functional_class,
        route_type=route_type,
        is_accessible_by_pedestrians=raw_lg.get('is_accessible_by_pedestrians'),
        is_built_up_area_road=raw_lg.get('is_built_up_area_road'),
        is_controlled_access=raw_lg.get('is_controlled_access'),
        is_limited_access=raw_lg.get('is_limited_access'),
        is_multiply_digitized=raw_lg.get('is_multiply_digitized'),
        is_ramp=raw_lg.get('is_ramp'),
        is_tunnel=raw_lg.get('is_tunnel'),
        is_urban=raw_lg.get('is_urban'),
        is_within_interchange=raw_lg.get('is_within_interchange'),
        lane_segment_refs=[])

    return lane_group


def boundary_length(points, utm_zone):
    points_m = np.array([utm.from_latlon(lat, lng, force_zone_number=utm_zone)[:2]
                         for lng, lat in points])

    points_diff = np.diff(points_m, axis=0)
    dist = np.hypot(points_diff[:, 0], points_diff[:, 1])

    return dist.sum()


def convert_lane_to_geojson(raw_lane, tile_id, lane_group_id, lane_idx, start_connector_ref, end_connector_ref,
                            reverse_dot=False):
    assert raw_lane['type'] == 'lane', raw_lane['type']
    lane_num = lane_idx + 1
    center_line = raw_lane['pts']
    altitude_pts = raw_lane['altitude_pts']

    left_boundary_ref = map_types.create_lane_boundary_ref(tile_id, lane_group_id, raw_lane['left_boundary_id'])
    right_boundary_ref = map_types.create_lane_boundary_ref(tile_id, lane_group_id, raw_lane['right_boundary_id'])

    start_junction_ref = map_types.create_junction_ref_from_connector_ref(start_connector_ref,
                                                                          raw_lane['start_connector_lane_number'])
    end_junction_ref = map_types.create_junction_ref_from_connector_ref(end_connector_ref,
                                                                        raw_lane['end_connector_lane_number'])

    # we actually ignore the per lane direction of travel, and look at
    # the presumed DOT for the lane group instead
    dot = raw_lane['direction_of_travel']
    if reverse_dot:
        # we should only be reversing backwards or unknown segments
        assert dot in {'BACKWARD', 'NONE'}

        # swap the junctions
        start_junction_ref, end_junction_ref = end_junction_ref, start_junction_ref

        # also swap left and right boundaries
        left_boundary_ref, right_boundary_ref = right_boundary_ref, left_boundary_ref

        # reverse the center line
        center_line.reverse()

        altitude_pts.reverse()

        dot = 'FORWARD'

    lane_seg_ref = map_types.create_lane_segment_ref(tile_id, lane_group_id, raw_lane['id'])

    lane_type = raw_lane['lane_type']
    assert lane_type in VALID_LANE_TYPES

    lane_transition_type = raw_lane['lane_transition_type']
    assert lane_transition_type in VALID_LANE_TRANSITION_TYPES, lane_transition_type

    lane_seg = geojson_utils.create_feature(
        'lane',
        lane_seg_ref,
        geojson.LineString(center_line),
        left_boundary_ref=left_boundary_ref,
        right_boundary_ref=right_boundary_ref,
        start_junction_ref=start_junction_ref,
        end_junction_ref=end_junction_ref,
        lane_num=lane_num,
        upcoming_split=raw_lane['upcoming_split'],
        is_transitioning=raw_lane['is_transitioning'],
        is_emergency_lane=raw_lane['is_emergency_lane'],
        altitude_pts=altitude_pts,
        merging=raw_lane['merging'],
        from_split=raw_lane['from_split'],
        direction_of_travel=dot,
        lane_transition_type=lane_transition_type,
        lane_type=lane_type,
        merged=raw_lane['merged'])

    return lane_seg


def convert_boundary_to_geojson(raw_boundary, tile_id, lane_group_id, reverse_dot=False):
    assert raw_boundary['type'] == 'lane_boundary', raw_boundary['type']

    lane_boundary_ref = map_types.create_lane_boundary_ref(tile_id, lane_group_id, raw_boundary['id'])

    style = raw_boundary['style']
    assert style in {None, 'UNKNOWN', 'SOLID', 'DASHED'}, style

    color = raw_boundary['color']
    assert color in {None, 'UNKNOWN', 'YELLOW', 'WHITE'}, color

    material = raw_boundary['material']
    assert material in {None, 'UNKNOWN', 'PAINT', 'BARRIER', 'BOTTS_DOTS'}, material

    line = raw_boundary['pts']

    if reverse_dot:
        line.reverse()

    boundary = geojson_utils.create_feature(
        'lane_boundary',
        lane_boundary_ref,
        geojson.LineString(line),
        style=style,
        color=color,
        marking_width=raw_boundary['marking_width'],
        material=material,
        is_only_emergency_boundary=raw_boundary['is_only_emergency_boundary'],
        altitudes=raw_boundary['altitude_pts'])

    return boundary


def convert_connector_to_geojson(raw_conn, tile_id):
    assert raw_conn['type'] == 'connector', raw_conn['type']

    connector_ref = map_types.create_connector_ref(tile_id, raw_conn['id'])

    connector = geojson_utils.create_feature(
        'connector',
        connector_ref,
        geojson.LineString(raw_conn["boundary_geometry"]),
        connected_lane_groups=raw_conn['connected_lane_groups'],
        inflow_refs=raw_conn.get('inflow_refs', []),
        outflow_refs=raw_conn.get('outflow_refs', []))

    return connector


def convert_junction_to_geojson(raw_junction, tile_id, connector_id):
    assert raw_junction['type'] == 'junction', raw_junction['type']

    junction_ref = map_types.create_junction_ref(tile_id, connector_id, raw_junction['id'])
    junction = geojson_utils.create_feature(
        'junction',
        junction_ref,
        geojson.Point(raw_junction['pt']),
        junction_type=raw_junction['junction_type'],
        inflow_refs=raw_junction.get('inflow_refs', []),
        outflow_refs=raw_junction.get('outflow_refs', []))

    return junction


# -------------------------------------------
# Helper Methods
# -------------------------------------------

def determine_direction_of_travel(raw_lg):
    # find the DOT for all non-emergency lanes (which have NONE) in this lane group
    all_dot = {l['direction_of_travel'] for l in raw_lg['lanes'] if not l['is_emergency_lane']}

    if len(all_dot) == 1:
        # a single direction
        dot = all_dot.pop()
    else:
        # confused lane group, contains both directions
        dot = 'UNKNOWN'

    return dot
