"""
A simple library for creating more descriptive ref ids from the simple
ids and hierarchy of the original here maps data.
"""


class HashableDict(dict):
    """
    Add a hashing function for dicts. This allows us to use refs as
    fully index-able ids.
    """
    def __hash__(self):
        return hash(tuple(sorted(self.values())))


def hashify(obj):
    """
    Recursively convert all dicts to HashableDicts. Does a deep search
    into all lists and dicts.

    :param obj: the object to be hashified
    :return: the updated object
    """
    if isinstance(obj, HashableDict):
        return obj
    elif isinstance(obj, list):
        return [hashify(i) for i in obj]
    elif isinstance(obj, dict):
        return HashableDict({key: hashify(value) for key, value in obj.iteritems()})
    return obj


def create_lane_group_ref(tile_id, lane_group_id):
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'lane_group_ref',
        'tile_id': tile_id,
        'id': lane_group_id
    })


def lane_group_ref_from_lane_ref(lane_ref):
    """
    Convert a lane ref into its parent lane group ref

    :param lane_ref: the full lane.ref of the lane
    :return: a hashable ref key for the lane group
    """
    return hashify(
        {'id': lane_ref['lane_group_id'],
         'tile_id': lane_ref['tile_id'],
         'type': 'lane_group_ref'})


def create_lane_segment_ref(tile_id, lane_group_id, lane_segment_id):
    assert 0 <= lane_segment_id < (1 << 8), lane_segment_id
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'lane_ref',
        'tile_id': tile_id,
        'lane_group_id': lane_group_id,
        'id': lane_segment_id
    })


def create_connector_ref(tile_id, connector_id):
    assert 0 <= connector_id < (1 << 64), connector_id
    assert 0 <= tile_id < (1 << 64), "tile_id: {}, type(tile_id): {}".format(
            tile_id, type(tile_id))

    return hashify({
        'type': 'connector_ref',
        'tile_id': tile_id,
        'id': connector_id
    })


def create_junction_ref(tile_id, connector_id, junction_id):
    assert 0 <= junction_id < (1 << 8), junction_id
    assert 0 <= connector_id < (1 << 64), connector_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'junction_ref',
        'tile_id': tile_id,
        'connector_id': connector_id,
        'id': junction_id
    })


def create_junction_ref_from_connector_ref(connector_ref, junction_id):
    return create_junction_ref(connector_ref['tile_id'],
                               connector_ref['id'],
                               junction_id)


def create_lane_boundary_ref(tile_id, lane_group_id, boundary_id):
    assert 0 <= boundary_id < (1 << 8), boundary_id
    assert 0 <= lane_group_id < (1 << 64), lane_group_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'lane_boundary_ref',
        'tile_id': tile_id,
        'lane_group_id': lane_group_id,
        'id': boundary_id
    })


def create_road_segment_ref(tile_id, road_segment_id):
    assert 0 <= road_segment_id < (1 << 64), road_segment_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'road_segment_ref',
        'tile_id': tile_id,
        'id': road_segment_id
    })


def create_road_connector_ref(tile_id, road_connector_id):
    assert 0 <= road_connector_id < (1 << 64), road_connector_id
    assert 0 <= tile_id < (1 << 64), tile_id

    return hashify({
        'type': 'road_connector_ref',
        'tile_id': tile_id,
        'id': road_connector_id
    })


def lidar_segment_ref(section_id, segment_id, y_cluster, sub_cluster_id=0):
    return hashify({
        'type': 'lidar_segment_ref',
        'section_id': section_id,
        'segment_id': segment_id,
        'y_cluster': y_cluster,
        'sub_cluster_id': sub_cluster_id,
    })


def lidar_spline_ref(section_id, group_id):
    return hashify({
        'type': 'lidar_spline_ref',
        'section_id': section_id,
        'group_id': group_id
    })
