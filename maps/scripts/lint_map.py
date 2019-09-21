#! /usr/bin/env python
""" Script to lint the map for potential errors. """
import argparse
import os
import glob
import rospkg

from maps.linting import route_linter
from maps.utils import emblog
emblog.set_ros(False)

parser = argparse.ArgumentParser("Lint all da lanes")
parser.add_argument("--route_id", action='append', dest="route_ids", help="a list of routes to lint")
parser.add_argument("--map_dir", default=None, help="input dir for tiles, default to lane_map_server")
parser.add_argument("--map_reader_dir", default=None, help="dir of map reader files")
parser.add_argument("--out_file", default=None, help="file to write issue layer output")
parser.add_argument("--issue_type", dest="issue_types", action='append', default=None,
                    help="list of issue types to search for")


def main():
    args = parser.parse_args()

    map_dir = args.map_dir
    if map_dir is None:
        map_dir = os.path.join(rospkg.RosPack().get_path('lane_map_server'), 'maps/tiled_maps/usa')

    map_reader_dir = args.map_reader_dir
    if map_reader_dir is None:
        map_reader_dir = os.path.join(rospkg.RosPack().get_path('map_reader'), 'maps')

    routes_dir = os.path.join(rospkg.RosPack().get_path('brain'), 'params/routes')
    route_ids = args.route_ids
    if route_ids is None:
        route_ids = []
        # demap_dirfault to all routes
        for fn in glob.glob(os.path.join(routes_dir, '*.yaml')):
            route_id = os.path.splitext(os.path.basename(fn))[0]
            route_ids.append(route_id)

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
    else:
        emblog.debug("\nLinting PASSED.\n")


if __name__ == '__main__':
    main()
