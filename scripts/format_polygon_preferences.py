#! /usr/bin/env python

"""This file is designed to help format polygons that may be generated using something like geojson.io and add refs
to them so they work in the linter.
"""

import argparse
import os

import geojson


class IDTracker:
    def __init__(self, taken_set=None, begin_id=0):
        if taken_set is None:
            taken_set = set()
        self.__current_id = begin_id
        self.__taken_set = taken_set

    def get_next_id(self):
        while self.__current_id in self.__taken_set:
            self.__current_id += 1
        self.__taken_set.add(self.__current_id)
        return self.__current_id

    def add_taken_id(self, new_id):
        self.__taken_set.add(new_id)


def reassign_polygon_ids(feature_set, route_id):
    id_tracker = IDTracker()

    for feature in feature_set['features']:
        if 'ref' in feature['properties']:
            id_tracker.add_taken_id(feature['properties']['ref']['id'])

    for feature in feature_set['features']:
        if 'ref' not in feature['properties'] or 'ref' not in feature:
            ref = {'id': id_tracker.get_next_id(),
                   'geometry_type': 'Polygon',
                   'route_id': route_id}
            feature['properties']['ref'] = ref
            feature['ref'] = ref


def strip_geojson_io_formatting(feature_set):
    geojson_formatting_properties = ('stroke', 'stroke-width', 'stroke-opacity', 'fill', 'fill-opacity')
    for prop in geojson_formatting_properties:
        for feature in feature_set['features']:
            if prop in feature['properties']:
                del feature['properties'][prop]


def strip_refs(feature_set):
    for feature in feature_set['features']:
        if 'ref' in feature:
            del feature['ref']
        if 'ref' in feature['properties']:
            del feature['properties']['ref']


def fix_preferred_lanes_type(feature_set):
    """
    Sometimes geojson.io will make the preferred_lanes field a string "[n]" instead of [n]
    :param feature_set:
    :return:
    """
    for feature in feature_set['features']:
        for prop in ('preferred_lanes', 'lanes_to_avoid'):
            if prop in feature['properties'] and type(feature['properties'][prop]) is str:
                feature['properties'][prop] = eval(feature['properties'][prop])


def load_geojson(filename):
    with open(filename, 'r') as json_file:
        return geojson.load(json_file)


def save_geojson(feature_set, filename):
    with open(filename, 'w') as json_file:
        geojson.dump(feature_set, json_file, indent=4)


def format_file(filename):
    feature_set = load_geojson(filename)
    route_id = os.path.splitext(os.path.basename(filename))[0]
    reassign_polygon_ids(feature_set, route_id)
    strip_geojson_io_formatting(feature_set)
    fix_preferred_lanes_type(feature_set)
    save_geojson(feature_set, filename)
    print "Formatted", filename


def clean_file(filename):
    feature_set = load_geojson(filename)
    strip_geojson_io_formatting(feature_set)
    strip_refs(feature_set)
    fix_preferred_lanes_type(feature_set)
    save_geojson(feature_set, filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='+', type=str, help="A list of Polygon geojson files that may not have refs")
    parser.add_argument('--clean-only', dest='clean_only', action='store_true')
    args = parser.parse_args()

    if args.clean_only:
        action = clean_file
    else:
        action = format_file

    for file_name in args.files:
        action(file_name)
