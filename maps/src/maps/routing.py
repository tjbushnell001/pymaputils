""" Utility functions for routing through map tiles. """

import shapely.geometry
import heapq
import geopy.distance
import maps
from maps.utils import geojson_utils

MPH_TO_MPS = 0.44704
FREEWAY_SPEED = 65 * MPH_TO_MPS
CITY_SPEED = 25 * MPH_TO_MPS


def point_to_bbox(point):
    return point.x, point.y, point.x, point.y


def find_nearest(road_graph, feature_type, point):

    tile_id = maps.utils.tile_utils.lat_lng_to_tile_id(point.y, point.x, road_graph.tile_level)
    tile = road_graph.get_tile(tile_id)
    if tile is None:
        return None

    for feature in tile.get_features(feature_type).itervalues():
        # do not consider invalid features
        if feature.properties.get("invalid", False):
            continue

        boundary = shapely.geometry.shape(feature.geometry)

        if boundary.intersects(point):
            return feature

    return None


def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path


def get_neighbors(road_graph, current):
    seg = road_graph.get_feature(current)

    current_is_ramp = seg.properties['is_ramp']

    connector_ref = geojson_utils.hashify(seg.properties['end_connector_ref'])

    # make sure junction tile is loaded
    connector = road_graph.get_feature(connector_ref)

    # valid segments should have valid junctions
    if connector is None:
        return
    # assert connector is not None

    neighbor_refs = map(geojson_utils.hashify, connector.properties['outflow_refs'])
    for neighbor_ref in neighbor_refs:
        # make sure neighbors are loaded
        neighbor = road_graph.get_feature(neighbor_ref)

        # do not route over invalid route segments
        if neighbor.properties['invalid']:
            continue

        # start with the actual length of the road segment
        dist = neighbor.properties['length']

        # ballpark speed limit
        if neighbor.properties['is_controlled_access']:
            speed_limit = FREEWAY_SPEED
        else:
            speed_limit = CITY_SPEED

        # get cost in seconds
        cost = dist / speed_limit

        # penalize ramps by adding 30 seconds
        if neighbor.properties['is_ramp']:
            cost += 30.0

        yield neighbor_ref, cost


def heuristic_cost_estimate(map_tiles, src_id, dest_id):
    src_tile = map_tiles.get_tile(src_id['tile_id'])
    src_seg = src_tile.get_features('road_segment').get(src_id)
    assert src_seg is not None

    dest_tile = map_tiles.get_tile(dest_id['tile_id'])
    dest_seg = dest_tile.get_features('road_segment').get(dest_id)
    assert dest_seg is not None

    src_seg_end = src_seg.properties['right_boundary'][-1][:2]
    dest_seg_end = dest_seg.properties['right_boundary'][-1][:2]

    dist = geopy.distance.distance(src_seg_end, dest_seg_end).meters

    # estimate cost in seconds, assuming freeway speeds
    # NOTE: it's desirable to under estimate cost in the heuristic
    # (http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html)
    cost = dist / FREEWAY_SPEED

    return cost


def a_star(road_graph, start_id, goal_id, ignore_set=None):
    # The set of nodes already evaluated
    closedSet = set()
    if ignore_set is not None:
        closedSet.update(ignore_set)
        if start_id in closedSet:
            closedSet.remove(start_id)

    # The set of currently discovered nodes that are not evaluated yet.
    # Initially, only the start node is known.
    openSet = {start_id}

    # For each node, which node it can most efficiently be reached from.
    # If a node can be reached from many nodes, cameFrom will eventually contain the
    # most efficient previous step.
    cameFrom = {}

    # For each node, the cost of getting from the start node to that node.
    gScore = {}

    # The cost of going from start to start is zero.
    gScore[start_id] = 0

    # For each node, the total cost of getting from the start node to the goal
    # by passing by that node. That value is partly known, partly heuristic.
    fScore = {}

    # For the first node, that value is completely heuristic.
    fScore[start_id] = heuristic_cost_estimate(road_graph, start_id, goal_id)
    min_fScore = [(fScore[start_id], start_id)]

    while openSet:
        current = None
        while len(min_fScore) > 0 and current is None:
            # the node in openSet having the lowest fScore value
            fs, current = heapq.heappop(min_fScore)
            if current not in openSet or fs != fScore[current]:
                current = None

        if current is None:
            return None, gScore

        elif current == goal_id:
            return reconstruct_path(cameFrom, current), {}

        openSet.remove(current)
        closedSet.add(current)
        del fScore[current]

        for neighbor_id, neighbor_dist in get_neighbors(road_graph, current):

            if neighbor_id in closedSet:
                # Ignore the neighbor which is already evaluated.
                continue

            # The distance from start to a neighbor
            tentative_gScore = gScore[current] + neighbor_dist

            if neighbor_id not in openSet:
                # Discover a new node
                openSet.add(neighbor_id)
            elif tentative_gScore >= gScore[neighbor_id]:
                continue

            # This path is the best until now. Record it!
            cameFrom[neighbor_id] = current
            gScore[neighbor_id] = tentative_gScore
            fScore[neighbor_id] = gScore[neighbor_id] + heuristic_cost_estimate(road_graph, neighbor_id, goal_id)
            heapq.heappush(min_fScore, (fScore[neighbor_id], neighbor_id))

    return None, gScore


def coord_to_lat_lng(point):
  return (point[1], point[0])

def find_route(road_graph, waypoints, capabilities):
    routes = []
    route = []
    route_wps = []
    closed_set = set()

    allowed_ramps = set()

    prev_segment_id = None
    prev_wp_id = None
    prev_wp = None
    for wp in waypoints:
        wp_id = wp.ref
        point = shapely.geometry.shape(wp.geometry)

        waypoint_type = wp.properties['waypoint_type']

        # make sure waypoint tile is loaded
        road_segment = find_nearest(road_graph, 'road_segment', point)

        if road_segment is None:
            # issues.add_issue(wp, "Can't find segment for waypoint")
            print "Can't find segment for waypoint", wp_id
            return None

        segment_id = road_segment.ref

        if waypoint_type == 'allowed_ramp':
            # these aren't _real_ waypoints, but they instead authorize us to take certain ramps
	    # we need to keep track of them for later
            allowed_ramps.add(segment_id)
            continue

        if prev_segment_id is None:
            if waypoint_type not in ('trip_origin', 'sub_origin', 'reroute_origin'):
                print "Not an origin waypoint", wp_id
                return None

            # start route segment
            route_wps.append(wp_id)
            route.append(segment_id)
            prev_segment_id = segment_id
            prev_wp_id = wp_id
            prev_wp = wp
            continue

        if prev_segment_id == segment_id:
            # same segment. skip it
            continue

        sub_route, progress = a_star(road_graph, prev_segment_id, segment_id, ignore_set=closed_set)

        if sub_route is None:
            print "No route from waypoint", prev_wp_id, prev_segment_id, coord_to_lat_lng(prev_wp.geometry['coordinates']), "to", wp_id, segment_id, coord_to_lat_lng(wp.geometry['coordinates'])

            # print out the furthest point the router was able to reach, which is sometimes a useful debugging hint
            if len(progress) > 0:
                furthest_ref, furthest_sec = max(progress.items(), key=lambda x: x[1])
                furthest = road_graph.get_feature(furthest_ref)
                coords = furthest.properties['left_boundary'][-1]
                furthest_min = int(furthest_sec / 60)
                print "Furthest: [{}] {:d}h:{:02d}m {}".format(furthest_ref, furthest_min / 60, furthest_min % 60, coord_to_lat_lng(coords))

            return None

        assert route[-1] == sub_route[0]

        closed_set.update(sub_route[:-1])
        route.extend(sub_route[1:])
        route_wps.append(wp_id)

        if waypoint_type in ('trip_destination', 'sub_destination'):
            routes.append((route, route_wps))
            route = []
            route_wps = []
            prev_segment_id = None
            prev_wp_id = None
            prev_wp = None
            continue

        prev_segment_id = segment_id
        prev_wp_id = wp_id
        prev_wp = wp

    # check for incomplete route
    if len(route) > 0:
        print "Route must end on a destination waypoint"
        return None

    # make sure we didn't route across any non allowed ramps
    # we could restrict the route so that it only takes allowed
    # ramps, however this can be hard to debug.
    # instead, we disincentive the router to take ramps in general,
    # and expect it to only resort to them when absolutely nessecary.
    # then, we reject any routes that use non-allowed ramps
    if capabilities.whitelist_ramps:
        ramp_error = False
        for route, _ in routes:
            prev_is_ramp = None
            for rs_ref in route:
                rs = road_graph.get_feature(rs_ref)
                rs_is_ramp = rs.properties['is_ramp']

                # we only care about transitions from non-ramps onto ramps
                if (rs_is_ramp and
                    prev_is_ramp == False and
                    rs_ref not in allowed_ramps):
                    coords = rs.properties['left_boundary'][-1]
                    print "Error: Ramp [{}] is not allowed. {}".format(rs_ref, coord_to_lat_lng(coords))
                    ramp_error = True

                prev_is_ramp = rs_is_ramp
        if ramp_error:
            return None

    return routes
