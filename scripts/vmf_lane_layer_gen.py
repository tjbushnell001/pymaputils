import argparse
import csv
import geojson
import os
import shapely.geometry as sg
import numpy as np
import math
import matplotlib.pyplot as plt
import copy

from maps.lane_maps import ConvertedLaneMapLayer
from maps.feature_dict import FeatureDict
from maps.map_layers import MapLayers
from maps.map_types import MapType
from maps.road_graph import ROAD_GRAPH_TILE_LEVEL
from maps.utils import tile_utils
from maps.utils import routing_utils
from maps.utils import emblog
from maps.routing import find_route
from maps.utils import map_conversions as mc
from maps.utils import geojson_utils
from maps.utils import ref_utils

from shapely.ops import substring as sg_substring


MAP_ENUM_TO_STRING = {
	0: "LINE",
	1: "CIRCLE",
	2: "CLOTHOID"
}


def parse_args(args=None):
	parser = argparse.ArgumentParser(description="Example lane map data parsing script to find lane group tha intersects a point.")
	parser.add_argument('data_file', type=str, help="The csv dictating lane group splitting.")
	parser.add_argument('--map_dir', type=str, default="~/tiled_maps",
					  help="The location of the tiled_maps directory. Defaults to embite storage location.")
	return parser.parse_args(args=args)


def split_lane_group(lane_map, lane_group, num_segments, utm_zone):
	lane_refs = lane_group.properties['lane_segment_refs']
	lanes = [copy.deepcopy(lane_map.get_feature(lane_ref)) for lane_ref in lane_refs]

	if len(lanes) == 0:
		print 'error no lanes:', lane_group.ref

	boundaries = set()
	transitioning = False
	for lane in lanes:
		if lane.properties['merging'] or lane.properties['from_split']:
			transitioning = True

		boundaries.add(lane.properties['left_boundary_ref'])
		boundaries.add(lane.properties['right_boundary_ref'])

	boundaries = [copy.deepcopy(lane_map.get_feature(boundary_ref)) for boundary_ref in boundaries]

	if transitioning:
		return lanes, boundaries
	else:
		convert_features_to_utm(lanes, utm_zone)
		convert_features_to_utm(boundaries, utm_zone)

		# left_boundary = min(boundaries, key=lambda b: b['ref']['id'])
		# right_boundary = max(boundaries, key=lambda b: b['ref']['id'])

		lane_features = []
		for lane in lanes:
			new_lanes = split_line(lane, num_segments)
			for i, new_lane in enumerate(new_lanes):
				new_lg_id = int("{}{}{}".format(lane_group.ref['id'], 0, i))
				lane_features.append(geojson_utils.create_feature(
					feature_type=lane.feature_type,
					feature_ref=ref_utils.hashify({
						"type": lane.ref['type'],
						"id": lane.ref['id'],
						"tile_id": lane.ref['tile_id'],
						"lane_group_id": new_lg_id
					}),
					geometry=geojson.LineString(new_lane),
					base_lane_group_id=lane_group.ref['id'],
					**lane.properties
				))
			if len(new_lanes) == 0:
				raise ValueError("empty lanes: {}".format(lane_group.ref))

		boundary_features = []
		for sub_id, boundary in enumerate(boundaries):
			new_boundaries = split_line(boundary, num_segments)
			for i, new_boundary in enumerate(new_boundaries):
				new_lg_id = int("{}{}{}".format(lane_group.ref['id'], 0, i))
				boundary_features.append(geojson_utils.create_feature(
					feature_type=boundary.feature_type,
					feature_ref=ref_utils.hashify({
						"type": boundary.ref['type'],
						"id": boundary.ref['id'],
						"tile_id": boundary.ref['tile_id'],
						"lane_group_id": new_lg_id
					}),
					geometry=geojson.LineString(new_boundary),
					base_lane_group_id=lane_group.ref['id'],
					sub_id=(sub_id + 1),
					**boundary.properties
				))
			if len(new_boundaries) == 0:
				raise ValueError("empty new_boundaries: {}".format(lane_group.ref))

		lane_features = convert_features_to_lon_lat(lane_features, utm_zone)
		boundary_features = convert_features_to_lon_lat(boundary_features, utm_zone)

		return lane_features, boundary_features


def split_line(line, num_segments):
	line_ls = sg.LineString(line.geometry.coordinates)
	ls_dist = line_ls.length / float(num_segments)

	segments = []
	for i in range(num_segments):
		start = i * ls_dist
		end = (i + 1) * ls_dist
		segment = sg_substring(line_ls, start, end).coords
		segments.append(segment)
	return segments

def convert_features_to_lon_lat(features, utm_zone):
	for feature in features:
		points = []
		for point in feature.geometry.coordinates:
			lat_lon = mc.convert_utm_to_lat_lon(point[0], point[1], utm_zone)
			points.append([lat_lon[1], lat_lon[0]])
		feature.geometry.coordinates = points
	return features

def convert_features_to_utm(features, utm_zone):
	for feature in features:
		points = []
		for point in feature.geometry.coordinates:
			points.append(mc.convert_lat_lon_to_utm(point[1], point[0], utm_zone))
		feature.geometry.coordinates = points
	return features


def create_vmf_features(lane_group_rows, lane_map, utm_zone):
	tile_id = int(float(lane_group_rows[0][-4]))
	lg_id = int(float(lane_group_rows[0][-3]))
	lane_group_ref = ref_utils.create_lane_group_ref(tile_id, lg_id)
	lane_group = lane_map.get_feature(lane_group_ref)

	boundary_model_map = dict()

	max_segments = 1
	for lg_row in lane_group_rows:
		if max_segments < int(float(lg_row[0])):
			max_segments = int(float(lg_row[0]))

		boundary_id = int(float(lg_row[-2]))
		boundary_ref = ref_utils.create_lane_boundary_ref(tile_id, lg_id, boundary_id)
		if boundary_ref not in boundary_model_map:
			boundary_model_map[boundary_ref] = dict()
		sub_id = int(float(lg_row[-1]))
		boundary_model_map[boundary_ref][sub_id] = int(float(lg_row[5]))

	lanes, boundaries = split_lane_group(lane_map, lane_group, max_segments, utm_zone)

	new_lg_ids = set()
	for boundary in boundaries:
		model_type = "UNKNOWN"
		if boundary.ref['lane_group_id'] not in new_lg_ids:
			new_lg_ids.add(boundary.ref['lane_group_id'])
		old_boundary_ref = ref_utils.create_lane_boundary_ref(tile_id, lg_id, boundary.ref['id'])
		if old_boundary_ref in boundary_model_map:
			if 'sub_id' not in boundary.properties:
				# print 'no sub_id in boundary', boundary.ref, '?'
				continue
			else:
				if boundary.properties['sub_id'] in boundary_model_map[old_boundary_ref]:
					model_enum = boundary_model_map[old_boundary_ref][boundary.properties['sub_id']]
				else:
					closest_id = boundary_model_map[old_boundary_ref].keys()[0]
					for sub_id in boundary_model_map[old_boundary_ref].keys()[1:]:
						if abs(sub_id - boundary.properties['sub_id']) < abs(closest_id - boundary.properties['sub_id']):
							closest_id = sub_id
					model_enum = boundary_model_map[old_boundary_ref][closest_id]
				model_type = MAP_ENUM_TO_STRING[model_enum]
		else:
			# No model description for this lane. Set to UNKNOWN?
			# print '*****************************'
			# print 'no model for lane boundary:',  old_boundary_ref
			# print '*****************************'
			continue
		boundary.properties['model_type'] = model_type

	new_lane_groups = []
	for new_lg_id in new_lg_ids:
		new_lane_group = copy.deepcopy(lane_group)
		new_lane_group.ref['id'] = new_lg_id
		new_lane_groups.append(new_lane_group)		

	return lanes + boundaries + new_lane_groups


def main(args):
	data_file = os.path.abspath(os.path.expanduser(args.data_file))
	map_dir = os.path.abspath(os.path.expanduser(args.map_dir))

	# map data is nestled inside the usa subdir in tiled_maps
	usa_dir = os.path.join(map_dir, 'usa')
	map_layers = MapLayers(map_dir=usa_dir)

	lane_map = map_layers.get_layer(MapType.LANE, cache_tiles=True)
	new_lane_map = ConvertedLaneMapLayer(os.path.join(usa_dir, 'vmf_tiles'), cache_tiles=False, fix_dot=False)

	tile_features = []
	lane_group_rows = []
	tile_lg_map = dict()

	i = 0
	first_row = False
	current_tile_id = 0

	print 'Reading csv file...'
	with open(data_file) as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		for row in reader:
			if not first_row:
				first_row = True
			else:
				lg_ref = ref_utils.create_lane_group_ref(int(float(row[-4])), int(float(row[-3])))

				if lg_ref['tile_id'] not in tile_lg_map:
					tile_lg_map[lg_ref['tile_id']] = dict()
				tile_dict = tile_lg_map[lg_ref['tile_id']]

				if lg_ref not in tile_dict:
					tile_dict[lg_ref] = []
				tile_dict[lg_ref].append(row)

	print 'Finished reading csv, starting processing...'
	for tile_id in tile_lg_map:
		tile_features = []
		tile_dict = tile_lg_map[tile_id]
		for lg_ref in tile_dict:
			rows = tile_dict[lg_ref]
			tile_id = lg_ref['tile_id']
			utm_zone = tile_utils.tile_utm_zone(tile_id, 14)
			tile_features += create_vmf_features(rows, lane_map, utm_zone)

		# Save the new lg features in the tile
		if new_lane_map.tile_exists(tile_id):
			new_tile = new_lane_map.get_tile(tile_id)
			for feature in tile_features:
				new_tile.add_feature(feature, overwrite=True)
		else:
			new_tile = geojson_utils.create_feature_collection('tile', tile_id, tile_features)
			new_tile = FeatureDict(new_tile)
		
		new_lane_map.save_tile(tile_id, new_tile)

		i += 1
		if i % 10 == 0:
		   	print '{} tiles complete!'.format(i)
		print 'Finished tile:', tile_id


if __name__ == "__main__":
	args = parse_args()
	main(args)
