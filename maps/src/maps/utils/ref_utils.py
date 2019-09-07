"""
A simple library for creating more descriptive ref id's from the simple id's and hierarchy of the original here
maps data.
"""
from maps import feature_layer


def create_lane_group_ref(tile_id, lane_group_id):
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify({'type': 'lane_group_ref', 'tile_id': tile_id, 'id': lane_group_id})


def create_lane_segment_ref(tile_id, lane_group_id, lane_segment_id):
    assert 0 <= lane_segment_id < (1 << 8), lane_segment_id
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify(
        {'type': 'lane_ref', 'tile_id': tile_id, 'lane_group_id': lane_group_id, 'id': lane_segment_id})


def create_connector_ref(tile_id, connector_id):
    assert 0 <= connector_id < (1 << 64), connector_id
    assert 0 <= tile_id < (1 << 64), "tile_id: {}, type(tile_id): {}".format(tile_id, type(tile_id))

    return feature_layer.hashify({'type': 'connector_ref', 'tile_id': tile_id, 'id': connector_id})


def create_junction_ref(tile_id, connector_id, junction_id):
    assert 0 <= junction_id < (1 << 8), junction_id
    assert 0 <= connector_id < (1 << 64), connector_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify(
        {'type': 'junction_ref', 'tile_id': tile_id, 'connector_id': connector_id, 'id': junction_id})


def create_junction_ref_from_connector_ref(connector_ref, junction_id):
    return create_junction_ref(connector_ref['tile_id'],
                               connector_ref['id'],
                               junction_id)


def create_lane_boundary_ref(tile_id, lane_group_id, boundary_id):
    assert 0 <= boundary_id < (1 << 8), boundary_id
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify(
        {'type': 'lane_boundary_ref', 'tile_id': tile_id, 'lane_group_id': lane_group_id, 'id': boundary_id})


def create_road_segment_ref(tile_id, road_segment_id):
    assert 0 <= road_segment_id < (1 << 64), road_segment_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify({'type': 'road_segment_ref', 'tile_id': tile_id, 'id': road_segment_id})


def create_road_connector_ref(tile_id, road_connector_id):
    assert 0 <= road_connector_id < (1 << 64), road_connector_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return feature_layer.hashify({'type': 'road_connector_ref', 'tile_id': tile_id, 'id': road_connector_id})
