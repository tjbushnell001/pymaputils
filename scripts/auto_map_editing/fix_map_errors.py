import copy
import geopy
import geopy.distance
import json
import math
import numpy as np
import os
from geographiclib.geodesic import Geodesic

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

base_dir = '/Users/ranreichman/Downloads/'
map_dir = '/Users/ranreichman/src/brain/perception/lane_map_server/maps/tiled_maps/usa/tiles'

def get_bearing(p1, p2):
    geodesic_val = Geodesic.WGS84.Inverse(p1[0], p1[1], p2[0], p2[1])
    return geodesic_val['azi1']

def getPointAngle(p1, p2):
    radians = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    return math.degrees(radians)

def getAngle(lat1, long1, lat2, long2):
    radians = math.atan2(long2 - long1, lat2 - lat1)
    return math.degrees(radians)

def getKinkAngle(p1, p2, p3):
    try:
        a = np.array(p1)
        b = np.array(p2)
        c = np.array(p3)

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)
    except:
        return 90

    return np.degrees(angle)

def coordinateDistanceInMeters(c1, c2):
    distance_factor = math.pow(10, 6) / 11

    x_distance = distance_factor * (c1[0] - c2[0])
    y_distance = distance_factor * (c1[1] - c2[1])

    return math.sqrt(math.pow(x_distance,2) + math.pow(y_distance,2))

def getNewLocation(latitude, longitude, heading, distance):
    start = geopy.Point(latitude, longitude)
    d = geopy.distance.distance(meters=distance)

    dest = d.destination(point=start, bearing=heading)
    return [dest.latitude, dest.longitude]

class MapFixer(object):
    def __init__(self, range_of_seconds_to_fix, point_max_distance_in_m):
        self.loadTiles()
        self.update_strings = set()
        self.updated_tiles = set()
        self.saved_timestamps = []
        self.range_of_seconds_to_fix = range_of_seconds_to_fix
        self.point_max_distance_in_m = point_max_distance_in_m
        self.tiles_and_lane_groups_fixed = {}

    def loadTiles(self):
        self.tiles = {}
        print("loading tiles")
        for filename in os.listdir(map_dir):
            if filename.endswith(".json"):
                with open(map_dir + "/" + filename, "r") as json_file:
                    tile_data = json.load(json_file)
                    self.tiles[filename] = tile_data
        print("tiles loaded")

    def isTimestampInRangeOfFixedTimestamp(self, timestamp):
        for fix_timestamp in self.timestamps_to_fix:
            if abs(timestamp - fix_timestamp) < self.range_of_seconds_to_fix:
                return True

        return False

    def getRunIdRawData(self, run_id):
        filename = base_dir + str(run_id) + "_bad_timestamps.csv"
        raw_data = []

        with open(filename, "r") as f_in:
            lines = f_in.readlines()
            header_line = lines[0]
            header_line_splitted = header_line.replace("\n", "").split(",")

            header_to_column = {}
            for i in range(len(header_line_splitted)):
                header_to_column[i] = header_line_splitted[i]

            for line in lines[1:]:
                line = line.replace("\n", "")
                splitted = [float(x) for x in line.split(",")]
                row_val = {}
                for i in range(len(splitted)):
                    row_val[header_to_column[i]] = splitted[i]

                if self.isTimestampInRangeOfFixedTimestamp(row_val['timestamp']):
                    raw_data.append(row_val)

        return raw_data

    # The most accurate diff is one with the lowest x difference.
    def getMostAccurateDiffPerTimestamp(self, raw_data):
        most_accurate_diff = {}
        for row in raw_data:
            x_difference = abs(row['map_point_x'] - row['camera_point_x'])
            timestamp = row['timestamp']

            if timestamp in most_accurate_diff and x_difference > most_accurate_diff[timestamp]['x_difference']:
                continue

            most_accurate_diff[timestamp] = {"value" : row, "x_difference" : x_difference}

        return most_accurate_diff

    def findRelevantTiles(self, current_location):
        relevant_tile_names = []
        for tile_filename in self.tiles:
            tile_data = self.tiles[tile_filename]
            tile_representative_location_reversed = tile_data['connectors'][0]['boundary_geometry'][0]
            tile_representative_location = [tile_representative_location_reversed[1],
                                            tile_representative_location_reversed[0]]
            coordinate_distance = coordinateDistanceInMeters(current_location, tile_representative_location)
            if coordinate_distance < 2000:
                print(coordinate_distance)
                print(tile_filename)
                relevant_tile_names.append(tile_filename)

        return relevant_tile_names

    # It should only be one lane but if it's 'from_split' and they're in the same lane group then we choose both lanes.
    def getLanesToEdit(self, relevant_tile_names, current_location):
        current_location_point = Point(current_location[0], current_location[1])
        for tile_name in relevant_tile_names:
            tile_data = self.tiles[tile_name]
            for lane_group_index, lane_group in enumerate(tile_data['lane_groups']):
                lane_group_points = lane_group['left_boundary'] + lane_group['right_boundary']
                lane_group_polygon = Polygon([(x[1], x[0]) for x in lane_group_points])
                if lane_group_polygon.contains(current_location_point):
                    for i in range(len(lane_group['boundaries']) - 1):
                        lane_points = lane_group['boundaries'][i]['pts'] + lane_group['boundaries'][i+1]['pts']
                        lane_points_polygon = Polygon([(x[1], x[0]) for x in lane_points])
                        if lane_points_polygon.contains(current_location_point):
                            lane_points = lane_group['lanes'][i]['pts']

                            lane_ids = [i]
                            if lane_group['lanes'][i]['from_split']:
                                for j in range((len(lane_group['boundaries']) - 1)):
                                    if lane_group['lanes'][j]['from_split'] and j not in lane_ids:
                                        lane_ids.append(j)
                            if not (lane_group['lanes'][i]['lane_transition_type'] == "MERGE"):
                                tile_id = int(tile_name.split(".")[0])
                                if tile_id not in self.tiles_and_lane_groups_fixed:
                                    self.tiles_and_lane_groups_fixed[tile_id] = []
                                self.tiles_and_lane_groups_fixed[tile_id].append(lane_group['id'])
                                return tile_name, lane_group_index, lane_ids, [[x[1], x[0]] for x in lane_points]

        raise Exception

    def getClosestPointLaneIndex(self, lane_points, current_location):
        closest_lane_point_index = None
        closest_lane_point_distance = None
        for i, lane_point in enumerate(lane_points):
            distance = coordinateDistanceInMeters(current_location, lane_point)
            if closest_lane_point_distance is None or closest_lane_point_distance > distance:
                closest_lane_point_index = i
                closest_lane_point_distance = distance

        return closest_lane_point_index

    def getNewLocationWithHorizontalOffset(self, start_point, end_point, offset_to_move_point):
        bearing = getAngle(start_point[0], start_point[1], end_point[0], end_point[1])
        new_bearing = bearing - 90 % 360
        return getNewLocation(end_point[0], end_point[1], new_bearing, offset_to_move_point)

    def updateLanePoints(self, lane_points, closest_lane_index, offset_to_move_point):
        if closest_lane_index > 0:
            index_start = closest_lane_index - 1
            index_end = closest_lane_index
        else:
            index_start = closest_lane_index
            index_end = closest_lane_index + 1

        start_point = lane_points[index_start]
        end_point = lane_points[index_end]

        new_location = self.getNewLocationWithHorizontalOffset(start_point, end_point, offset_to_move_point)

        print("Previous location: " + str(lane_points[closest_lane_index]))
        print("New location: " + str(new_location))
        lane_points[closest_lane_index] = new_location
        return lane_points

    def getUpdateString(self, tile_name, lane_group_index, lane_id, closest_lane_point_index):
        return str(tile_name) + "-" + str(lane_group_index) + "-" + str(lane_id) + "-" + str(closest_lane_point_index)

    def addMissingLanePointsToPath(self, lane_points):
        new_lane_points = [lane_points[0]]

        prev_lane_point = lane_points[0]

        for lane_point in lane_points[1:]:
            distance_between_points = coordinateDistanceInMeters(prev_lane_point, lane_point)
            added_points = []
            if distance_between_points > self.point_max_distance_in_m:
                number_of_added_points = int(distance_between_points // self.point_max_distance_in_m)
                distance_between_added_points = distance_between_points / (number_of_added_points + 1)
                bearing = get_bearing(prev_lane_point, lane_point)
                for i in range(number_of_added_points):
                    new_location = \
                        getNewLocation(prev_lane_point[0], prev_lane_point[1], bearing, distance_between_added_points)
                    # This is used so that we can later notice places where we added points
                    new_location_rounded_to_seven_digits = [round(new_location[0], 7), round(new_location[1], 7)]
                    added_points.append(new_location_rounded_to_seven_digits)
                    prev_lane_point = new_location

            new_lane_points += added_points
            new_lane_points.append(lane_point)

            prev_lane_point = lane_point

        return new_lane_points

    def fixKinksInLanePoints(self, lane_points):
        if len(lane_points) < 3:
            return lane_points

        original_lane_points = copy.deepcopy(lane_points)

        ANGLE_THRESHOLD_FOR_KINK_IN_DEGREES = 2
        ANGLE_INDICATING_FAILURE_IN_DEGREES = 10
        KINK_FIX_OFFSET = 0.04

        have_kinks = True
        while have_kinks:
            have_kinks = False
            for i in range(len(lane_points) - 2):
                first_point_in_tri = lane_points[i]
                second_point_in_tri = lane_points[i + 1]
                third_point_in_tri = lane_points[i + 2]

                kink_angle = getKinkAngle(first_point_in_tri, second_point_in_tri, third_point_in_tri)
                angle_diff = 180 - kink_angle

                # If we're above a certain angle we were unable to fix
                if angle_diff > ANGLE_INDICATING_FAILURE_IN_DEGREES:
                    return original_lane_points

                if abs(angle_diff) > ANGLE_THRESHOLD_FOR_KINK_IN_DEGREES:
                    have_kinks = True
                    direction = angle_diff / abs(angle_diff)
                    lane_points[i + 1] = \
                        self.getNewLocationWithHorizontalOffset(first_point_in_tri, second_point_in_tri, -direction * KINK_FIX_OFFSET)
                    lane_points[i + 2] = \
                        self.getNewLocationWithHorizontalOffset(second_point_in_tri, third_point_in_tri,
                                                                direction * KINK_FIX_OFFSET)
                    break

        return lane_points

    def applyUpdate(self, update_string, lane_points, closest_lane_point_index, raw_row_data, tile_name,
                    lane_group_index, lane_ids):
        for lane_id in lane_ids:
            if update_string not in self.update_strings:
                new_lane_points = self.updateLanePoints(lane_points, closest_lane_point_index, raw_row_data["diff"])

                new_lane_points = self.fixKinksInLanePoints(new_lane_points)

                new_lane_points_reversed = [[x[1], x[0]] for x in new_lane_points]

                tile_data = self.tiles[tile_name]
                tile_data['lane_groups'][lane_group_index]['lanes'][lane_id]['pts'] = new_lane_points_reversed
                self.tiles[tile_name] = tile_data
                self.updated_tiles.add(tile_name)
                self.update_strings.add(update_string)

    def updateClosestPathPoint(self, raw_row_data):
        try:
            # We want to move forward in our current heading because the map/camera error is ahead of us
            current_location = getNewLocation(raw_row_data['latitude'], raw_row_data['longitude'],
                                              raw_row_data['heading'], raw_row_data["map_point_x"])
            relevant_tile_names = self.findRelevantTiles(current_location)
            tile_name, lane_group_index, lane_ids, lane_points = \
                self.getLanesToEdit(relevant_tile_names, current_location)

            lane_points = self.addMissingLanePointsToPath(lane_points)
            closest_lane_point_index = self.getClosestPointLaneIndex(lane_points, current_location)

            update_string = self.getUpdateString(tile_name, lane_group_index, lane_ids, closest_lane_point_index)
            self.applyUpdate(update_string, lane_points, closest_lane_point_index, raw_row_data, tile_name,
                             lane_group_index, lane_ids)
        except:
            print("failed with row: " + str(raw_row_data))

        self.saved_timestamps.append(raw_row_data['timestamp'])

    def fixLocationsForTimetamps(self, most_accurate_results_per_timestamp):
        sorted_timestamps = sorted(most_accurate_results_per_timestamp.keys())

        for timestamp in sorted_timestamps:
            self.updateClosestPathPoint(most_accurate_results_per_timestamp[timestamp]["value"])

        for timestamp in self.saved_timestamps:
            print("Updated timestamp successfully: " + str(timestamp))

        for tile_name in self.updated_tiles:
            with open(map_dir + "/" + tile_name, "w") as json_file:
                print("dumping tile " + str(tile_name))
                json_to_dump = json.dumps(self.tiles[tile_name], indent=4, separators=(', ',': '))
                json_file.write(json_to_dump)
                print("dumped tile " + str(tile_name))

    def getFixLocationsAndOffset(self, run_id, timestamps_to_fix):
        self.timestamps_to_fix = timestamps_to_fix

        raw_data = self.getRunIdRawData(run_id)
        most_accurate_results_per_timestamp = self.getMostAccurateDiffPerTimestamp(raw_data)
        self.fixLocationsForTimetamps(most_accurate_results_per_timestamp)

if __name__ == '__main__':
    point_max_distance_in_m = 20
    range_of_seconds_to_fix = 8
    # Only fix if within X seconds of a timestamp
    runs_to_timestamps_to_fix = {
        18543: [1598918705],
        18545: [1598922427],
        18560: [1598996680, 1598997185, 1598999336, 1598999453],
        18621: [1599142616, 1599143081, 1599143142],
        18627: [1599147866],
        18628: [1599148595, 1599148922, 1599150023, 1599150258],
        18632: [1599156635, 1599156692],
        18633: [1599158252, 1599158414, 1599159106],
        18635: [1599174529],
        18639: [1599180318],
        18651: [1599237451],
        18653: [1599246395, 1599246448],
        18670: [1599342588],
        18673: [1599354995]
    }
    map_fixer = MapFixer(range_of_seconds_to_fix, point_max_distance_in_m)

    for run_id in runs_to_timestamps_to_fix:
        print("working on run id " + str(run_id))
        timestamps_to_fix = runs_to_timestamps_to_fix[run_id]
        locations_and_offsets = map_fixer.getFixLocationsAndOffset(run_id, timestamps_to_fix)

    print(map_fixer.tiles_and_lane_groups_fixed)