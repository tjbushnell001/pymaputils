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

        left_boundary = boundary_features[left_line_ref].geometry["coordinates"]
        right_boundary = boundary_features[right_line_ref].geometry["coordinates"]
        poly = geojson_utils.boundaries_to_poly(left_boundary, right_boundary)
        if poly.contains(Point(lng, lat)):
            return lane
    return None


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
        tile_id =  key["tile_id"]
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
    tile_id =  lane_key_list[highway_num]["tile_id"]
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
    tile_id = tile_utils.lat_lng_to_tile_id(lat, lng, tile_level)
    cached_tile = cache.get(tile_id)
    if cached_tile is None:
        tile = lane_maps.get_tile(tile_id)
        if tile is None:
            print "have a bad map reference"
            assert(False)
        cache.set(tile_id, tile)
    else:
        tile = cached_tile
    current_lane = find_lane(lat, lng, tile)
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

