from enum import Enum

from maps.utils import ref_utils


FORWARD_TRANSITION_PRIORITY = {
    'UNKNOWN': 0,
    'MERGE': 1,
    'SPLIT': 2,
    None: 3,  # INVALID
}


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


def get_next_merge_lane(lane_map, lane_ref):
    lane = lane_map.get_feature(lane_ref)

    while True:
        lane = load_next_lane(lane, lane_map)
        if not lane:
            return None

        if lane["properties"]["merging"]:
            return lane


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

    def filter_lanes(lane):
        if lane is None:
            return False

        return (include_emergency_lanes or
                not lane["properties"]["is_emergency_lane"])

    next_lanes = filter(filter_lanes, next_lanes)

    likely_lanes = get_nominal_lanes(lane_map, next_lanes)
    if not likely_lanes:
        return

    return likely_lanes[0]
