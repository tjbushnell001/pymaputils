#!/usr/bin/env python
import math
import numpy as np
import time
import utm
from maps.utils import geojson_utils
from maps.utils import ref_utils
from maps.utils import tile_utils
from shapely.geometry import Point
from maps.utils import lru_cache
from scipy.spatial import cKDTree

#TODO rename or use an includable class
class Entry:
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Lane:
    ref_lane_line = []
    left_lane_line = []
    right_lane_line = []

    def __init__(self):
        self.x = []
        self.y = []
        self.z = []

    def __add__(self, other):
        result = Lane()
        # assumes lanes are sorted on input
        result.ref_lane_line = self.ref_lane_line + other.ref_lane_line
        result.left_lane_line = self.left_lane_line + other.left_lane_line
        result.right_lane_line = self.right_lane_line + other.right_lane_line
        return result

    def to_json(self):
        ref_xs = []
        ref_ys = []

        left_xs = []
        left_ys = []

        right_xs = []
        right_ys = []

        for entry in self.ref_lane_line:
            ref_xs.append(entry.x)
            ref_ys.append(entry.y)

        for entry in self.left_lane_line:
            left_xs.append(entry.x)
            left_ys.append(entry.y)

        for entry in self.right_lane_line:
            right_xs.append(entry.x)
            right_ys.append(entry.y)

        result = {'ref_line_xs': ref_xs,
                  'ref_line_ys': ref_ys,
                  'left_lane_line_xs': left_xs,
                  'left_lane_line_ys': left_ys,
                  'right_lane_line_xs': right_xs,
                  'right_lane_line_ys': right_ys
                  }
        return result


# finds the lane that a given lat, lng point is in on a given title
# assumes that only one lane falls on a tile
def find_lane(lat, lng, tile):
    lane_features = tile.get_features('lane')
    boundary_features = tile.get_features('lane_boundary')

    for lane in lane_features.itervalues():
        if lane.properties['is_emergency_lane']:
            continue

        left_line_ref = lane.properties["left_boundary_ref"]
        right_line_ref = lane.properties["right_boundary_ref"]

        # CHANGE TO BE BETWEEN LEFT AND RIGHT LINES (SEE
        # std::unordered_set<lane_map::LaneRef> associateLanes(const maps::LaneSubMap& map,
        #                                                      const geometry_msgs::Point& object))
        left_boundary = boundary_features[left_line_ref].geometry["coordinates"]
        right_boundary = boundary_features[right_line_ref].geometry["coordinates"]
        poly = geojson_utils.boundaries_to_poly(left_boundary, right_boundary)
        if poly.contains(Point(lng, lat)):
            return lane
    return None


def find_closest_lane(lat, lng,  lane_maps, tile_level, cache):
    """
    Using a KD-tree find the lane with center closest to lat,lng heading
    :param lat:
    :param lng:
    :param lane_maps:
    :param tile_level:
    :param cache:
    :return:
    """
    t0_setup = time.time()
    current_tile_id = tile_utils.lat_lng_to_tile_id(lat, lng, tile_level)
    tile_ids = tile_utils.adjacent_tile_ids(current_tile_id, tile_level, include_self=True)

    lane_line_indices = []
    lane_indices = []
    lane_pts = []
    lane_index = 0
    lane_line_index = 0
    lane_index_to_lane = {}

    get_lane_data_time = 0.0
    to_np_lists_time = 0.0
    get_feature_time = 0.0
    cache_lookup_time = 0.0

    num_valid_tiles = 0
    for tile_id in tile_ids:

        t0_cache_lookup = time.time()
        # THIS IS THE SLOW SECTION ( lane_maps.get_tile() )
        cached_tile = cache.get(tile_id)
        if cached_tile is None:
            tile = lane_maps.get_tile(tile_id)
            if tile is None:
                continue
            cache.set(tile_id, tile)
        else:
            tile = cached_tile
        num_valid_tiles += 1
        cache_lookup_time += (time.time()-t0_cache_lookup)*1000

        # prepare data for lookup -- each lane pnt has a mapping back to which lane it's from
        t0_get_feature = time.time()
        lane_features = tile.get_features('lane')
        boundary_features = tile.get_features('lane_boundary')
        get_feature_time += (time.time()-t0_get_feature)*1000

        for lane in lane_features.itervalues():
            if lane.properties['is_emergency_lane']:
                continue

            t0_lane_pts = time.time()
            left_line_ref = lane.properties["left_boundary_ref"]
            right_line_ref = lane.properties["right_boundary_ref"]
            left_boundary = boundary_features[left_line_ref].geometry["coordinates"]
            right_boundary = boundary_features[right_line_ref].geometry["coordinates"]

            left_pts = np.array(left_boundary)
            right_pts = np.array(right_boundary)

            t = (time.time()-t0_lane_pts)*1000
            get_lane_data_time += t

            t0_np_lists = time.time()
            lane_indices.append(lane_index * np.ones(np.size(left_boundary, 0), dtype=int))
            lane_line_indices.append(lane_line_index * np.ones(np.size(left_boundary, 0), dtype=int))
            lane_pts.append(left_pts)
            lane_index_to_lane[lane_index] = (lane, tile)
            lane_line_index += 1

            lane_indices.append(lane_index * np.ones(np.size(right_boundary, 0), dtype=int))
            lane_line_indices.append(lane_line_index * np.ones(np.size(right_boundary, 0), dtype=int))
            lane_pts.append(right_pts)
            lane_index_to_lane[lane_index] = (lane, tile)
            lane_line_index += 1

            lane_index += 1
            t = (time.time()-t0_np_lists)*1000
            to_np_lists_time += t

    t0_concat = time.time()
    lane_indices = np.concatenate(lane_indices)
    lane_line_indices = np.concatenate(lane_line_indices)
    lane_pts = np.concatenate(lane_pts)
    t_concat = (time.time()-t0_concat)*1000
    print "cache_lookup_time = ", cache_lookup_time, " ms"
    print "get_feature_time = ", get_feature_time, " ms"
    print "get_lane_data_time = ", get_lane_data_time, " ms"
    print "to_np_lists_time = ", to_np_lists_time, " ms"
    print "t_concat = ", t_concat, " ms"
    print "data setup time = ", time.time()-t0_setup, " s"

    assert num_valid_tiles > 0
    print "num_valid_tiles = ", num_valid_tiles
    kd_tree = cKDTree(lane_pts)

    # Do query
    earth_circumference = 40075000.0  # 40,075 km
    search_radius_degrees = 100.0/earth_circumference * 360
    found_indices = np.array(kd_tree.query_ball_point(np.array([lng, lat]), search_radius_degrees))

    # print "r = ", search_radius_degrees
    # print "found_indices = ", found_indices

    # d, i = kd_tree.query(np.array([lng, lat]))
    # print "d = ", d, " deg ~ ", d/360.0 * earth_circumference, "m"
    # print "query: ", lng, lat, " found: ", lane_pts[i]
    # utm_q = np.array(utm.from_latlon(lat, lng)[:2])
    # utm_f = np.array(utm.from_latlon(lane_pts[i][1], lane_pts[i][0])[:2])
    # print "dist = ", np.linalg.norm(utm_q-utm_f)

    if len(found_indices) == 0:
        return None, None

    # group by lane_line
    found_lane_lines = lane_line_indices[found_indices]
    found_lanes = lane_indices[found_indices]

    sorting = np.argsort(found_lane_lines)
    found_lane_lines = found_lane_lines[sorting]
    found_lanes = found_lanes[sorting]
    found_pts = (lane_pts[found_indices])[sorting]

    # go through lane lines in order, calculate distances to left and right
    distances_for_lane_index = {}

    unique_lane_lines = np.unique(found_lane_lines)
    crr = 0
    for i in range(0, len(unique_lane_lines)):
        if i+1 < len(unique_lane_lines):
            nxt = np.searchsorted(found_lane_lines, unique_lane_lines[i+1])
            curr_lane_line_pts = found_pts[crr:nxt]
        else:
            nxt = -1
            curr_lane_line_pts = found_pts[crr:]

        lane_index = found_lanes[crr]

        dx = curr_lane_line_pts[:, 0] - lng
        dy = curr_lane_line_pts[:, 1] - lat

        dist = np.min(dx**2 + dy**2)
        if lane_index not in distances_for_lane_index:
            distances_for_lane_index[lane_index] = []
        distances_for_lane_index[lane_index].append(dist)

        crr = nxt

    best_lane_dist = np.inf
    best_lane_index = None
    for k, v in distances_for_lane_index.iteritems():
        if len(v) == 2 and np.sum(v) < best_lane_dist:
            best_lane_dist = np.sum(v)
            best_lane_index = k

    return lane_index_to_lane[best_lane_index]


def find_dist_to_end(points, start_lat, start_lng):
    a_utm = utm.from_latlon(start_lat, start_lng)  # TODO deal with utm zones

    if len(points) == 0:
        # no points have no distance
        return 0.0

    last_point = points[len(points) - 1]
    b_utm = utm.from_latlon(last_point[1], last_point[0])
    assert (a_utm[2] == b_utm[2] and a_utm[3] == b_utm[3])
    dx = a_utm[0] - b_utm[0]
    dy = a_utm[1] - b_utm[1]
    return math.sqrt(dx * dx + dy * dy)


def find_dist_to_start(points, start_lat, start_lng):
    a_utm = utm.from_latlon(start_lat, start_lng)  # TODO deal with utm zones
    assert (len(points) > 0)
    first_point = points[0]
    b_utm = utm.from_latlon(first_point[1], first_point[0])
    assert (a_utm[2] == b_utm[2] and a_utm[3] == b_utm[3])
    dx = a_utm[0] - b_utm[0]
    dy = a_utm[1] - b_utm[1]
    return math.sqrt(dx * dx + dy * dy)


# lat, lng is the location to use for distance
def get_lane_from_lane_ref_in_gcs_frame(lane_key_list, lat, lng, low_x, high_x, lane_maps, cache):
    # base case
    if low_x >= 0 >= high_x:
        # we are not asking for any more length so we return an empty lane
        return Lane()

    highway_num = -1
    if len(lane_key_list) == 0:
        print "logic error could not find a lane"
        return Lane()
    for key_index in xrange(len(lane_key_list)):
        key = lane_key_list[key_index]
        tile_id = key["tile_id"]
        cached_tile = cache.get(tile_id)
        if cached_tile is None:
            tile = lane_maps.get_tile(tile_id)
            if tile is None:
                print "have a bad map reference"
                assert(False)
            cache.set(tile_id, tile)
        else:
            tile = cached_tile
        lane_group_tile = tile.get_features('lane_group')
        lane_group_ref = ref_utils.create_lane_group_ref(key["tile_id"], key["lane_group_id"])
        if not lane_group_tile[lane_group_ref].properties["is_ramp"]:
            highway_num = key_index

    if highway_num == -1:
        print "no candidate found for map lane"
        return Lane()
    tile_id = lane_key_list[highway_num]["tile_id"]
    cached_tile = cache.get(tile_id)
    if cached_tile is None:
        tile = lane_maps.get_tile(tile_id)
        if tile is None:
            print "have a bad map reference"
            assert(False)
        cache.set(tile_id, tile)
    else:
        tile = cached_tile
    lane_tile = tile.get_features('lane')
    lane_boundary_tile = tile.get_features('lane_boundary')
    this_element = lane_tile[lane_key_list[highway_num]]

    ref_points = this_element.geometry.coordinates
    right_ref = this_element.properties["right_boundary_ref"]
    left_ref = this_element.properties["left_boundary_ref"]

    left_boundary = lane_boundary_tile[left_ref].geometry["coordinates"]
    right_boundary = lane_boundary_tile[right_ref].geometry["coordinates"]

    prev_junction = this_element.properties["start_junction_ref"]
    next_junction = this_element.properties["end_junction_ref"]

    # get the tiles we need for next and prev
    tile_next_cache = cache.get(next_junction['tile_id'])
    if tile_next_cache is None:
        tile_next = lane_maps.get_tile(next_junction['tile_id'])
        cache.set(tile_next.id, tile_next)
    else:
        tile_next = tile_next_cache
    junction_tile_next = tile_next.get_features('junction')

    # TODO partial add (in case the requested distance is less than what we need)
    result = Lane()
    result.left_lane_line = left_boundary
    result.right_lane_line = right_boundary
    result.ref_lane_line = ref_points

    if next_junction in junction_tile_next.keys():
        next_dist = find_dist_to_end(ref_points, lat, lng)
        new_high_x = high_x - next_dist

        if new_high_x > 0.0:
            # go to the next link to get more points
            location = junction_tile_next[next_junction].geometry["coordinates"]
            outflow_key = junction_tile_next[next_junction].properties["outflow_refs"]
            result = result + get_lane_from_lane_ref_in_gcs_frame(outflow_key, location[1], location[0], 0.0,
                                                                  new_high_x, lane_maps, cache)

    tile_prev = cache.get(next_junction['tile_id'])
    if tile_prev is None:
        tile_prev = lane_maps.get_tile(next_junction['tile_id'])
        cache.set(tile_next.id, tile_next)
    junction_tile_prev = tile_prev.get_features('junction')

    if prev_junction in junction_tile_prev.keys():
        prev_dist = find_dist_to_start(ref_points, lat, lng)
        new_low_x = low_x + prev_dist

        if new_low_x < 0.0:
            location = junction_tile_prev[prev_junction].geometry["coordinates"]
            inflow_key = junction_tile_prev[prev_junction].properties["inflow_refs"]
            result = get_lane_from_lane_ref_in_gcs_frame(inflow_key, location[1], location[0], new_low_x, 0.0,
                                                         lane_maps, cache) + result

    # return the result
    return result


def get_map_lane_lines_for_point_in_lane_gcs_frame(lat, lng, low_x, high_x, lane_maps, tile_level, cache):
    # When we switch tiles, lanes that start in one tile might not go over to the next one.
    # We need to query lanes from adjacent tiles also...

    # tile_id = tile_utils.lat_lng_to_tile_id(lat, lng, tile_level)
    # cached_tile = cache.get(tile_id)
    # if cached_tile is None:
    #     tile = lane_maps.get_tile(tile_id)
    #     if tile is None:
    #         print "have a bad map reference"
    #         assert False
    #     cache.set(tile_id, tile)
    # else:
    #     tile = cached_tile

    # current_lane = find_lane(lat, lng, tile)
    current_lane, tile = find_closest_lane(lat, lng, lane_maps, tile_level, cache)
    # alt_lane = find_closest_lane(lat, lng, lane_maps, tile_level, cache)
    print "tile.id = ", tile.id
    if current_lane is not None:
        print "current_lane = ", current_lane["ref"]
    else:
        print "current_lane = None"
    # if alt_lane is not None:
    #     print "alt_lane = ", alt_lane["ref"]
    # else:
    #     print "alt_lane = None"

    if current_lane is None:
        print "current lane could not be found"
        # empty lane is returned
        result = Lane()
        return result

    lane_boundary_tile = tile.get_features('lane_boundary')

    ref_points = current_lane.geometry.coordinates
    right_line_ref = current_lane.properties["right_boundary_ref"]

    left_line_ref = current_lane.properties["left_boundary_ref"]

    left_boundary = lane_boundary_tile[left_line_ref].geometry["coordinates"]
    right_boundary = lane_boundary_tile[right_line_ref].geometry["coordinates"]

    prev_junction = current_lane.properties["start_junction_ref"]
    next_junction = current_lane.properties["end_junction_ref"]

    # get the tiles we need for next and prev
    tile_next_cache = cache.get(next_junction['tile_id'])
    if tile_next_cache is None:
        tile_next = lane_maps.get_tile(next_junction['tile_id'])
        if tile_next is None:
            print "got a bad tile reference"
            assert(False)
        cache.set(tile_next.id, tile_next)
    else:
        tile_next = tile_next_cache
    junction_tile_next = tile_next.get_features('junction')

    # TODO partial add (in case the requested distance is less than what we need)
    result = Lane()
    result.left_lane_line = left_boundary
    result.right_lane_line = right_boundary
    result.ref_lane_line = ref_points
    if next_junction in junction_tile_next.keys():
        next_dist = find_dist_to_end(ref_points, lat, lng)
        new_high_x = high_x - next_dist

        if new_high_x > 0.0:
            # go to the next link to get more points
            location = junction_tile_next[next_junction].geometry["coordinates"]
            outflow_key = junction_tile_next[next_junction].properties["outflow_refs"]
            result = result + get_lane_from_lane_ref_in_gcs_frame(outflow_key, location[1], location[0], 0.0,
                                                                  new_high_x, lane_maps, cache)


    tile_prev_cached = cache.get(next_junction['tile_id'])
    if tile_prev_cached is None:
        tile_prev = lane_maps.get_tile(next_junction['tile_id'])
        if tile_prev is None:
            print "have a bad reference to a tile"
            assert(False)
        cache.set(tile_next.id, tile_next)
    else:
        tile_prev = tile_prev_cached
    junction_tile_prev = tile_prev.get_features('junction')

    if prev_junction in junction_tile_prev.keys():
        prev_dist = find_dist_to_start(ref_points, lat, lng)
        new_low_x = low_x + prev_dist

        if new_low_x < 0.0:
            location = junction_tile_prev[prev_junction].geometry["coordinates"]
            inflow_key = junction_tile_prev[prev_junction].properties["inflow_refs"]
            result = result + get_lane_from_lane_ref_in_gcs_frame(inflow_key, location[1], location[0], new_low_x, 0.0,
                                                                  lane_maps, cache)

    # return the result
    return result

# walks along a poly line and samples points at step distance from each other along the poly line
# assumes line is sorted
def sample_line(line, sample_min, sample_max, step):
    # assumes line is sorted
    xs = np.linspace(sample_min, sample_max, int((sample_max - sample_min) / step))
    result = []
    line_index = 0
    for x in xs:
        # find the value for given x
        while line_index < len(line):
            if line[line_index].x == x:
                this_entry = Entry()
                this_entry.x = x
                this_entry.y = line[line_index].y
                this_entry.z = line[line_index].z
                result.append(this_entry)
                break
            else:
                next_line_index = line_index + 1
                if next_line_index >= len(line):
                    # we have reached the end of the input line so we should return our results
                    return result

                if x < line[next_line_index].x:
                    # we found the segment for this point
                    dy = line[next_line_index].y - line[line_index].y
                    dx = line[next_line_index].x - line[line_index].x
                    this_entry = Entry()
                    this_entry.x = x
                    this_entry.y = line[line_index].y + dy * (x - line[line_index].x) / dx
                    this_entry.z = 0.0
                    result.append(this_entry)
                    break
                else:
                    line_index = line_index + 1
    return result

