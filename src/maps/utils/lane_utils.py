from enum import Enum

from maps.utils import ref_utils


FORWARD_TRANSITION_PRIORITY = {
    'UNKNOWN': 0,
    'MERGE': 1,
    'SPLIT': 2,
    None: 3,  # INVALID
}


class RelativeLane(Enum):
    LEFT_LEFT_ADJACENT      = -2
    LEFT_ADJACENT           = -1
    EGO                     = 0
    RIGHT_ADJACENT          = 1
    RIGHT_RIGHT_ADJACENT    = 2
    MERGE                   = 3
    LANE_CHANGE_SOURCE      = 4
    LANE_CHANGE_DESTINATION = 5


class Direction(Enum):
    FORWARD = 0
    BACKWARD = 1


def get_all_merge_lanes(lane_map, lane, max_results=5):
    if lane is None:
        return []

    output_lanes = [lane]

    while len(output_lanes) < max_results:
        start_junction_ref = lane["properties"]["start_junction_ref"]
        start_junction = lane_map.get_feature(start_junction_ref)
        if start_junction is None:
            break

        inflow_refs = start_junction["properties"]["inflow_refs"]

        if not inflow_refs:
            break

        for inflow_ref in inflow_refs:
            lane = lane_map.get_feature(inflow_ref)

            if len(output_lanes) < max_results:
                output_lanes.append(lane)

    return output_lanes


def follow_lanes(lane_map, initial_lane_ref, max_results=2):
    lane = lane_map.get_feature(initial_lane_ref)
    if lane is None:
        return []

    output_lanes = [lane]

    while len(output_lanes) < max_results:
        end_junction_ref = lane["properties"]["end_junction_ref"]
        end_junction = lane_map.get_feature(end_junction_ref)
        if end_junction is None:
            break

        outflow_refs = end_junction["properties"]["outflow_refs"]

        if not outflow_refs:
            break

        for outflow_ref in outflow_refs:
            lane = lane_map.get_feature(outflow_ref)

            if len(output_lanes) < max_results:
                output_lanes.append(lane)

    return output_lanes


def get_next_merge_ramp(lane_map, lane_ref):
    merge_lane = get_next_merge_lane(lane_map, lane_ref)
    if merge_lane is None:
        return None

    merge_junction = get_next_merge_junction(
        lane_map, lane_ref)
    if not merge_junction:
        return None

    merge_inflow_refs = merge_junction["properties"]["inflow_refs"]

    merge_inflow_refs.sort(key=lambda x: x["id"])
    for inflow_ref in merge_inflow_refs:
        inflow = lane_map.get_feature(inflow_ref)
        if inflow["properties"]["lane_transition_type"] == "MERGE":
            return inflow

    return None


def get_next_merge_lane(lane_map, lane_ref, max_iterations=5):
    """
    Returns the next merge lane, given a starting lane_ref.

    :param lane_map:
    :param lane_ref: lane ref of starting lane
    :param max_lanes: the maximum number of lanes ahead to look
    :return: next merge lane
    """
    lane = lane_map.get_feature(lane_ref)

    iterations = 0
    while lane and iterations < max_iterations:
        if lane["properties"]["merging"]:
            merge_junction = lane_map.get_feature(
                lane["properties"]["end_junction_ref"])
            merge_inflow_refs = merge_junction["properties"]["inflow_refs"]

            for merge_inflow_ref in merge_inflow_refs:
                lg_ref = ref_utils.lane_group_ref_from_lane_ref(merge_inflow_ref)
                is_ramp = lane_map.get_feature(lg_ref)["properties"]["is_ramp"]

                inflow = lane_map.get_feature(merge_inflow_ref)
                inflow_transition = inflow["properties"]["lane_transition_type"]
                is_lane_reduction = inflow_transition == "MERGE"
                if is_ramp or is_lane_reduction:
                    # We have an on-ramp or a lane reduction. This takes
                    # rightmost merge lane.
                    return lane_map.get_feature(merge_inflow_ref)

        lane = load_next_lane(lane, lane_map)
        iterations += 1

    return None


def get_next_merge_junction(lane_map, lane_ref):
    lane = lane_map.get_feature(lane_ref)

    while True:
        end_junction_ref = lane["properties"]["end_junction_ref"]
        end_junction = lane_map.get_feature(end_junction_ref)

        # This lane is merging, so return the junction.
        if lane["properties"]["merging"]:
            return end_junction

        if not end_junction or not end_junction["properties"]:
            return None

        outflow_refs = end_junction["properties"]["outflow_refs"]
        if not outflow_refs:
            return None

        for outflow_ref in outflow_refs:
            lane = lane_map.get_feature(outflow_ref)


# TODO: Add unit test to confirm behavior is the same as existing cpp function
def get_nominal_lanes(lane_map, candidate_lanes):
    """
    Given a lane map and a set of potential lanes, return the most likely
    continuation of the current lane.

    :param lane_map: The lane map MapLayer
    :param candidate_lanes: a list of lane objects.
    """
    lane_priorities = []
    for lane in candidate_lanes:
        lg_ref = ref_utils.lane_group_ref_from_lane_ref(lane.ref)
        lg = lane_map.get_feature(lg_ref)
        if lg is None:
            print 'MISSING LANE GROUP: %s' % lg_ref
            continue

        priority = (lg.properties['is_ramp'],
                    FORWARD_TRANSITION_PRIORITY[lane.properties['lane_transition_type']],
                    lg_ref['tile_id'], lg_ref['id'], lane.properties['lane_num'])

        lane_priorities.append((lane, priority))
    lane_priorities.sort(key=lambda x: x[1])

    if not lane_priorities:
        return []

    top_priority = lane_priorities[0][1]

    results = []
    for lane, priority in lane_priorities:
        if priority[:2] != top_priority[:2]:
            break
        results.append(lane)

    return results


def load_next_lane(lane, lane_map, direction=Direction.FORWARD,
                   include_emergency_lanes=False):
    junction_str, flow_str = (('end_junction_ref', 'outflow_refs') if direction == Direction.FORWARD
                              else ('start_junction_ref', 'inflow_refs'))
    junction_ref = lane.properties[junction_str]
    junction = lane_map.get_feature(junction_ref)
    if junction is None:
        print 'MISSING JUNCTION: %s' % junction_ref
        return

    next_lanes = [lane_map.get_feature(ref) for ref in junction.properties[flow_str]]

    def filter_lanes(curr_lane):
        if curr_lane is None:
            return False

        return (include_emergency_lanes or
                not curr_lane["properties"]["is_emergency_lane"])

    next_lanes = filter(filter_lanes, next_lanes)

    likely_lanes = get_nominal_lanes(lane_map, next_lanes)
    if not likely_lanes:
        return

    return likely_lanes[0]


def get_lanes_from_lane_occupancy(lane_map, lane_occupancy, tile_id=None):
    """
    Return the lane objects from the lane_occupancy message, preserving order.
    """
    lanes = []
    for lane_ref in lane_occupancy.lane_refs:
        if tile_id and lane_ref['tile_id'] != tile_id:
            continue

        lanes.append(
            lane_map.get_feature(ref_utils.lane_ref_msg_to_dict(lane_ref)))

    return lanes


def add_lane_ref_to_dict(lane_map, lane_ref, relative_lane, lane_refs_dict):
    lg_ref = ref_utils.lane_group_ref_from_lane_ref(lane_ref)
    lg = lane_map.get_feature(lg_ref)
    if not lg['properties']['is_ramp']:
        lane_refs_dict[relative_lane] = lane_ref


def add_adjacent_lane_refs_to_dict(lane_map, lane_occupancy, ego_lane_ref,
                                   lane_refs_dict, left=True):
    """
    Adds lane refs of the (left and left left) OR (right and right right)
    of ego_lane_ref to the lane_refs_dict.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :param ego_lane_ref: the lane ref referencing ego lane
    :param lane_refs_dict: the dict from RelativeLane to lane ref under
                           construction
    :param left: whether we should add the left or right lane refs.
    """
    lane_ref = get_adjacent_lane_ref(
        lane_map, lane_occupancy, ego_lane_ref, left=left)
    if lane_ref:
        relative_lane = RelativeLane.RIGHT_ADJACENT
        if left:
            relative_lane = RelativeLane.LEFT_ADJACENT

        add_lane_ref_to_dict(lane_map, lane_ref, relative_lane, lane_refs_dict)

    outer_lane_ref = get_adjacent_lane_ref(
        lane_map, lane_occupancy, lane_ref, left=left)
    if outer_lane_ref:
        relative_lane = RelativeLane.RIGHT_RIGHT_ADJACENT
        if left:
            relative_lane = RelativeLane.LEFT_LEFT_ADJACENT

        add_lane_ref_to_dict(lane_map, outer_lane_ref, relative_lane, lane_refs_dict)


def add_merge_lane_refs_to_dict(lane_map, ego_lane_ref, lane_refs_dict):
    """
    Add merge lane refs to the lane_refs_dict.

    :param lane_map: lane map layer
    :param ego_lane_ref: the lane ref referencing ego lane
    :param lane_refs_dict: the dict from RelativeLane to lane ref under
                           construction
    """
    merge_lane = get_next_merge_lane(lane_map, ego_lane_ref)
    if not merge_lane:
        return

    lane_refs_dict[RelativeLane.MERGE] = merge_lane["ref"]


def get_lane_refs_dict(lane_map, lane_occupancy):
    """
    Returns a mapping from relative lane to lane ref.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :return: a map from RelativeLane to lane ref.
    """
    lane_refs_dict = {}
    ego_lane_ref = get_ego_lane_ref(lane_map, lane_occupancy)
    if not ego_lane_ref:
        return lane_refs_dict

    lane_refs_dict[RelativeLane.EGO] = ego_lane_ref

    add_adjacent_lane_refs_to_dict(
        lane_map, lane_occupancy, ego_lane_ref, lane_refs_dict, left=True)
    add_adjacent_lane_refs_to_dict(
        lane_map, lane_occupancy, ego_lane_ref, lane_refs_dict, left=False)
    add_merge_lane_refs_to_dict(lane_map, ego_lane_ref, lane_refs_dict)

    return lane_refs_dict


def get_adjacent_lane_ref(lane_map, lane_occupancy, lane_ref, left=True):
    """
    Returns the lane ref to the right or left of lane_ref, if it exists.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :param lane_ref: lane ref to use as a reference
    :param left: True if we want the left adjacent, False if we want right.
    :return: the adjacent lane ref.
    """
    if lane_ref is None:
        return None

    lane = lane_map.get_feature(lane_ref)
    if not lane:
        return None

    lane_num = lane.properties['lane_num']
    adjacent_lane_num = lane_num + (-1 if left else 1)

    lanes = get_lanes_from_lane_occupancy(
        lane_map, lane_occupancy, tile_id=lane_ref['tile_id'])

    for lane in lanes:
        if lane and lane.properties['lane_num'] == adjacent_lane_num:
            return lane['ref']

    return None


def get_adjacent_lane(lane_map, lane_occupancy, left=True):
    """
    Returns the left or right adjacent lane to ego, if it exists.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :param left: True if we want the left adjacent, False if we want right.
    :return: the adjacent lane.
    """
    ego_lane_ref = get_ego_lane_ref(lane_map, lane_occupancy)
    adjacent_lane_ref = get_adjacent_lane_ref(
        lane_map, lane_occupancy, ego_lane_ref, left=left)

    if not adjacent_lane_ref:
        return None

    return lane_map.get_feature(adjacent_lane_ref)


def get_rightmost_lane(lane_map, lane_occupancy, include_emergency_lanes=False):
    """
    Returns the rightmost lane in the current lane group.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :param include_emergency_lanes: whether emergency lane should be considered
               i.e. if True, returned rightmost lane could be an emergency
               lane, if False, returned rightmost lane must not be an
               emergency lane
    :return: rightmost lane (lane segment object)
    """
    lane_objects = get_lanes_from_lane_occupancy(lane_map, lane_occupancy)

    if len(lane_objects) < 1:
        return None

    if include_emergency_lanes:
        return lane_objects[-1]

    for lane in reversed(lane_objects):
        if lane['properties']['is_emergency_lane']:
            continue
        return lane

    return None


def get_rightmost_lane_boundary(lane_map,
                                lane_occupancy,
                                include_emergency_lanes=False):
    """
    Returns the rightmost lane boundary in the current lane group.

    :param lane_map: lane map layer
    :param lane_occupancy: lane occupancy msg
    :param include_emergency_lanes: whether emergency lane should be considered
               i.e. if True, returned rightmost lane could be an emergency
               lane, if False, returned rightmost lane must not be an
               emergency lane
    :return: rightmost lane boundary (lane boundary object)
    """
    rightmost_lane = get_rightmost_lane(
        lane_map, lane_occupancy, include_emergency_lanes)
    right_boundary_ref = rightmost_lane['properties']['right_boundary_ref']
    return lane_map.get_feature(right_boundary_ref)


def get_ego_lane_ref(lane_map, lane_occupancy):
    """
    Return the lane ref for the ego lane.
    """
    if lane_occupancy is None or not lane_occupancy.possible_ego_lanes:
        return None

    for possible_ego_lane_ref in lane_occupancy.possible_ego_lanes:
        lane_group = lane_map.get_feature(
            ref_utils.lane_group_ref_from_lane_ref(possible_ego_lane_ref))

        if not lane_group['properties']['is_ramp']:
            # Most of the time we're not on a merge ramp. This helps avoid
            # confusion in criss-crossing lanes.
            return ref_utils.lane_ref_msg_to_dict(possible_ego_lane_ref)

    # If all the lanes are ramps, just take the first.
    return ref_utils.lane_ref_msg_to_dict(lane_occupancy.possible_ego_lanes[0])
