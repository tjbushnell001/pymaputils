import shapely.geometry

from maps import map_types
from maps.utils import geojson_utils


def relink_lane_tile(tile_id, lane_map, unlinked_lane_map, save_tiles=True):
    """
    Relink lane tiles stored in raw here maps format.

    :param tile_id: the central tile to be relinked
    :param lane_map: the tiled lane map layer
    :param unlinked_lane_map: unlinked tiled lane map layer
    :param save_tiles: boolean whether to write the new tiles back to disk after relinking
    :return: None
    """
    tile = lane_map.get_tile(tile_id)
    if tile is None:
        # tile doesn't exist
        return

    tile_connectors = tile.get_features('connector')
    tile_junctions = tile.get_features('junction')

    # clear connectors
    for c in tile_connectors.values():
        c.properties['connected_lane_groups'] = []
        c.properties['inflow_refs'] = []
        c.properties['outflow_refs'] = []

    # clear junctions
    for j in tile_junctions.values():
        j.properties['inflow_refs'] = []
        j.properties['outflow_refs'] = []
        j.properties['emergency_lanes_only'] = True

    for adjacent_tile in unlinked_lane_map.yield_adjacent_tiles(tile_id):
        for lg in adjacent_tile.get_features('lane_group').values():
            lg_dot = lg.properties['direction_of_travel']
            # only link directional lane groups
            if lg_dot not in ('BACKWARD', 'FORWARD'):
                continue
            # make sure to reverse connectors for BACKWARD lane groups
            reverse_dot = lg_dot == 'BACKWARD'

            # relink start connector
            link_connector(lg, tile_id, tile_connectors, True, reverse_dot=reverse_dot)

            # relink end connector
            link_connector(lg, tile_id, tile_connectors, False, reverse_dot=reverse_dot)

        for ls in adjacent_tile.get_features('lane').values():
            # only link directional lanes
            lane_dot = ls.properties['direction_of_travel']
            if lane_dot not in ('BACKWARD', 'FORWARD'):
                continue
            # make sure to reverse junctions for BACKWARD lanes
            reverse_dot = lane_dot == 'BACKWARD'

            # relink start junction
            link_junction(ls, tile, True, reverse_dot=reverse_dot)

            # relink end junction
            link_junction(ls, tile, False, reverse_dot=reverse_dot)

    for c in tile_connectors.values():
        connected_lane_group_refs = set()
        for ref_type in ('inflow_refs', 'outflow_refs'):
            for lg_ref in c.properties[ref_type]:
                connected_lane_group_refs.add(lg_ref)
        c.properties['connected_lane_groups'] = list(connected_lane_group_refs)

        # sort connector fields
        c.properties['connected_lane_groups'].sort()
        c.properties['inflow_refs'].sort()
        c.properties['outflow_refs'].sort()

    for j in tile_junctions.values():
        # sort junction fields
        j.properties['inflow_refs'].sort()
        j.properties['outflow_refs'].sort()

    lane_map.add_tile(tile_id, tile)
    if save_tiles:
        lane_map.save_tile(tile_id, tile)


def relink_road_tile(tile_id, road_graph, unlinked_road_graph, save_tiles=True):
    """
    Relink a road graph tile, i.e. fill in connectors with their lane group refs

    :param tile_id: the id of the central tile being relinked
    :param road_graph: the base output of the relinked tiles
    :param unlinked_road_graph: a raw translation of the lane map to road tile sans linking. This isn't strictly
        necessary but conceptually it separates unlinked and linked tiles.
    :param save_tiles: boolean whether to write the new linked tiles to disk once created.
    :return: None
    """
    tile = unlinked_road_graph.get_tile(tile_id)
    if tile is None:
        # tile doesn't exist
        return None

    tile_connectors = tile.get_features('road_connector')

    # clear connectors
    for c in tile_connectors.values():
        c.properties['connected_lane_groups'] = []
        c.properties['inflow_refs'] = []
        c.properties['outflow_refs'] = []

    for adjacent_tile in unlinked_road_graph.yield_adjacent_tiles(tile_id):
        for rs in adjacent_tile.get_features('road_segment').values():
            rs_dot = rs.properties['direction_of_travel']
            # only link forward road segments (no DOT correction here)
            if rs_dot != 'FORWARD':
                continue

            # relink start connector
            link_connector(rs, tile_id, tile_connectors, True)

            # relink end connector
            link_connector(rs, tile_id, tile_connectors, False)

    # sort connector fields
    for c in tile_connectors.values():
        c.properties['connected_lane_groups'].sort()
        c.properties['inflow_refs'].sort()
        c.properties['outflow_refs'].sort()

    road_graph.add_tile(tile_id, tile)
    if save_tiles:
        road_graph.save_tile(tile_id, tile)


def link_connector(lane_group, tile_id, connectors, is_start_connector, reverse_dot=False):
    """
    Link a lane_group or road_segment connector, i.e. populate the inflow or outflow ref of the connector with the
    lane_group / road_segment ref.

    :param lane_group: this is either a lane_group or road_segment, both of which have the same structure wrt connectors
    :param tile_id: the id of the tile this segment is found in
    :param connectors: a set of connectors for this tile such that we can get the tile from the id
    :param is_start_connector: determines whether the segment is entering or leaving the connector
    :param reverse_dot: whether or not to reverse the direction if we are correcting dot
    :return: None
    """
    if is_start_connector:
        connector_ref = lane_group.properties['start_connector_ref']
        outflow = True
    else:
        connector_ref = lane_group.properties['end_connector_ref']
        outflow = False

    if reverse_dot:
        outflow = not outflow

    if connector_ref['tile_id'] != tile_id:
        # not for this tile
        return

    direction_key = 'outflow_refs' if outflow else 'inflow_refs'
    connector = connectors.get(connector_ref)
    if connector is not None:
        connector.properties[direction_key].append(lane_group.ref)


def link_junction(lane_segment, tile, is_start_junction, reverse_dot=False):
    """
    Link a junction to it's lane segment, i.e. add the lane segment ref. If the junction does not exist,
    create it.

    :param lane_segment: the lane being connected to the junction
    :param tile: the tile, used to get the junction list
    :param is_start_junction: whether the lane is entering or exiting this junction
    :param reverse_dot: whether to reverse the direction while fixing dot.
    :return:
    """
    tile_id = tile.tile.id

    lane_points = lane_segment.geometry.coordinates

    if is_start_junction:
        junction_ref = lane_segment.properties['start_junction_ref']
        junction_pt = lane_points[0]
        outflow = True
    else:
        junction_ref = lane_segment.properties['end_junction_ref']
        junction_pt = lane_points[-1]
        outflow = False

    if reverse_dot:
        outflow = not outflow

    emergency_lane = lane_segment.properties['is_emergency_lane']

    if junction_ref['tile_id'] != tile_id:
        # not for this tile
        return

    # make sure connector exists
    connector_ref = map_types.create_connector_ref(junction_ref['tile_id'], junction_ref['connector_id'])
    if connector_ref not in tile.get_features('connector'):
        # connector doesn't exist
        return

    junction = tile.get_features('junction').get(junction_ref)
    if junction is None:
        # create missing lane junction
        junction = geojson_utils.create_feature(
            'junction',
            junction_ref,
            shapely.geometry.Point(junction_pt),
            junction_type="",
            inflow_refs=[],
            outflow_refs=[])
        tile.add_feature(junction)

    direction_key = 'outflow_refs' if outflow else 'inflow_refs'
    junction.properties[direction_key].append(lane_segment.ref)
    if not emergency_lane:
        junction.properties['emergency_lanes_only'] = False
