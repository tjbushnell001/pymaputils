import argparse
import os
import shapely.geometry as sg

from maps.map_layers import MapLayers
from maps.map_types import MapType
from maps.utils import tile_utils


def parse_args(args=None):
	parser = argparse.ArgumentParser(description="Example lane map data parsing script to find lane group tha intersects a point.")
	parser.add_argument('--map_dir', type=str, default="~/tiled_maps",
						help="The location of the tiled_maps directory. Defaults to embite storage location.")
	parser.add_argument('--gps_lat', type=float, default=34.02034454, help="gps latitude point to search for intersection.")
	parser.add_argument('--gps_lon', type=float, default=-117.101569874, help="gps longitude point to search for intersection.")
	return parser.parse_args(args=args)


def main(args):
	map_dir = os.path.abspath(os.path.expanduser(args.map_dir))

	# map data is nestled inside the usa subdir in tiled_maps
	usa_dir = os.path.join(map_dir, 'usa')
	map_layers = MapLayers(map_dir=usa_dir)

	# If you're inside embite and you're running in a brain enviroment with roscore, you can just do
	# map_layers = MapLayers()

	# Get your lane data
	lane_map = map_layers.get_layer(MapType.LANE)

	# Example GPS point in (lat, long)
	GPS_POINT = [args.gps_lat, args.gps_lon]

	# This is to fetch the data around the point you want
	tile_id = tile_utils.lat_lng_to_tile_id(lat=GPS_POINT[0], lng=GPS_POINT[1], tile_level=14)
	tile = lane_map.get_tile(tile_id)

	# Get your lane groups
	lane_groups = tile.get_features('lane_group')

	for lane_group in lane_groups.values():
		polygon = sg.Polygon(lane_group.geometry.coordinates[0])
		if sg.Point(GPS_POINT).intersects(polygon):
			# Here are all the features of the lane group you fetched!
			print '-------------- Properties --------------'
			print lane_group.properties
			print '----------------------------------------'
			print 'is highway:', lane_group.properties['is_controlled_access']
			print 'is ramp:', lane_group.properties['is_ramp']
			break


if __name__ == "__main__":
	args = parse_args()
	main(args)
