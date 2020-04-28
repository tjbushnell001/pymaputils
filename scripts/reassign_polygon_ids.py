"""This file is designed to help format polygons that may be generated using something like geojson.io and add refs
to them so they work in the linter.
"""

import geojson
import argparse


def reassign_polygon_ids(feature_set):
    current_id = 0
    for feature in feature_set['features']:
        feature['ref'] = {'id': current_id,
                          'type': 'Polygon'}
        current_id += 1


def format_file(filename):
    feature_set = geojson.load(open(filename, 'r'))
    reassign_polygon_ids(feature_set)
    geojson.dump(feature_set, open(filename, 'w'), indent=4)


def format_files(filenames):
    for filename in filenames:
        format_file(filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='+', type=str, help="A list of Polygon geojson files that may not have refs")
    args = parser.parse_args()
    format_files(args.files)
