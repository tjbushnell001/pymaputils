#! /usr/bin/env python
""" Script to lint the map for potential errors. """
import argparse
import glob
import os
import sys

# Not the cleanest solution, but this will make sure to find the location of the map lib
# Note(jared) it makes me sad to add this back :(
sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'src'))
from maps.linting import route_linter
from maps.utils import emblog

emblog.set_ros(False)

parser = argparse.ArgumentParser("Lint all da maps")
parser.add_argument("--route_id", action='append', dest="route_ids", help="a list of routes to lint")
parser.add_argument("--map_dir", default=None, help="input dir for tiles, default to lane_map_server")
parser.add_argument("--out_file", default=None, help="file to write issue layer output")
parser.add_argument("--issue_type", dest="issue_types", action='append', default=None,
                    help="list of issue types to search for")


def main():
    args = parser.parse_args()

    map_dir = args.map_dir
    if map_dir is None:
        # If no dir provided, assume tiled_maps is mounted in home like in embite.
        map_dir = os.path.join(os.path.expanduser('~'), 'tiled_maps/usa')

    map_reader_dir = os.path.join(map_dir, 'map_reader')
    route_ids = set()
    for fn in glob.glob(os.path.join(map_reader_dir, '*.json')):
        route_id = os.path.splitext(os.path.basename(fn))[0]
        route_ids.add(route_id)

    issue_layer, failures = route_linter.lint_routes(map_dir, map_reader_dir, route_ids, args.issue_types)

    if args.out_file is not None:
        print
        print "Writing Issues to File [{}]".format(args.out_file)
        issue_layer.write(args.out_file)

    if failures:
        emblog.error("\n"
                     "**************************************\n"
                     "* Linting FAILED!\n"
                     "**************************************\n")
        exit(1)
    else:
        emblog.debug("\nLinting PASSED.\n")


if __name__ == '__main__':
    main()
