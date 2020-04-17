from enum import Enum

from maps.utils import emblog, ref_utils

FORWARD_TRANSITION_PRIORITY = {
    'UNKNOWN': 0,
    'MERGE': 1,
    'SPLIT': 2,
    None: 3,  # INVALID
}

BACKWARD_TRANSITION_PRIORITY = {
    'UNKNOWN': 0,
    'SPLIT': 1,
    'MERGE': 2,
    None: 3,  # INVALID
}

class Direction(Enum):
    FORWARD = 0
    BACKWARD = 1

def get_nominal_lanes(lane_map, candidate_lanes, direction=Direction.FORWARD):
    """
    Given a lane map and a set of potential lanes, return the most likely continuation of the
    current lane.

    :param lane_map: The lane map MapLayer
    :param candidate_lanes: A list of lane objects.
    :param direction: Which direction we're travelling relative to junction

    """
    if direction == Direction.FORWARD:
        transition_priority = FORWARD_TRANSITION_PRIORITY
    else:
        transition_priority = BACKWARD_TRANSITION_PRIORITY

    lane_priorities = []
    for lane in candidate_lanes:
        lg_ref = ref_utils.lane_group_ref_from_lane_ref(lane.ref)
        lg = lane_map.get_feature(lg_ref)
        if lg is None:
            emblog.error('MISSING LANE GROUP: %s' % lg_ref)
            continue



        priority = (lg.properties['is_ramp'],
                    transition_priority[lane.properties['lane_transition_type']],
                    lg_ref['tile_id'], lg_ref['id'], lane.properties['lane_num'])

        lane_priorities.append((lane, priority))
    lane_priorities.sort(key=lambda x: x[1])

    if len(lane_priorities) == 0:
        return []

    top_priority = lane_priorities[0][1]

    results = []
    for lane, priority in lane_priorities:
        if priority[:2] != top_priority[:2]:
            break
        results.append(lane)
    return results

def get_next_likely_lane(lane, lane_map, direction=Direction.FORWARD):
    if lane is None:
        return None

    if direction == Direction.FORWARD:
        junction_key, flow_key = 'end_junction_ref', 'outflow_refs'
    else:
        junction_key, flow_key = 'start_junction_ref', 'inflow_refs'

    junction_ref = lane.properties[junction_key]
    junction = lane_map.get_feature(junction_ref)
    if junction is None:
        # missing junction
        return None

    next_lanes = [lane_map.get_feature(ref) for ref in junction.properties[flow_key]]
    next_lanes = filter(None, next_lanes)

    likely_lanes = get_nominal_lanes(lane_map, next_lanes, direction=direction)
    if len(likely_lanes) == 0:
        return None

    return likely_lanes[0]

def follow_lanes(initial_lane, lane_map, next_fn):
    lane = initial_lane
    while lane is not None:
        yield lane
        lane = next_fn(lane, lane_map)
