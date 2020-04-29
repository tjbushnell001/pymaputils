#! /usr/bin/env python

"""This file is designed to help format polygons that may be generated using something like geojson.io and add refs
to them so they work in the linter.
"""

import argparse
import geojson
import os


def reassign_polygon_ids(feature_set, route_id):
    current_id = 0
    for feature in feature_set['features']:
        feature['ref'] = {'id': current_id,
                          'type': 'Polygon',
                          'route_id': route_id}
        current_id += 1


def format_file(filename):
    feature_set = geojson.load(open(filename, 'r'))
    route_id = os.path.splitext(os.path.basename(filename))[0]
    reassign_polygon_ids(feature_set, route_id)
    geojson.dump(feature_set, open(filename, 'w'), indent=4)
    print "Formatted", filename


def format_files(filenames):
    for filename in filenames:
        format_file(filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='+', type=str, help="A list of Polygon geojson files that may not have refs")
    args = parser.parse_args()
    format_files(args.files)
