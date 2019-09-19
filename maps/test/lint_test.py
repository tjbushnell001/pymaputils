#! /usr/bin/env python
""" Simple script to sift through lanes and verify they follow our constraints on lane labels. """
import os
import glob
import sys

from maps.linting import route_linter
from maps.utils import emblog

# -------------------------------------------------
# Because this lint test is not running in a ROS environment, we can't rely on our ros params or package locations
# for the map data.
# TODO: Make sure to keep these file locations up to date!
# -------------------------------------------------

BRAIN_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../..")
MAP_DIR = os.path.join(BRAIN_ROOT, "perception/lane_map_server/maps/tiled_maps/usa")
MAP_READER_DIR = os.path.join(BRAIN_ROOT, "perception/map_reader/maps")
ROUTES_DIR = os.path.join(BRAIN_ROOT, "brain/params/routes")


def main():
    route_ids = set()
    for fn in glob.glob(os.path.join(ROUTES_DIR, '*.yaml')):
        route_id = os.path.splitext(os.path.basename(fn))[0]
        route_ids.add(route_id)

    issue_layer, failures = route_linter.lint_routes(MAP_DIR, MAP_READER_DIR, route_ids)

    if failures:
        emblog.error("\n"
                     "**************************************\n"
                     "* Linting FAILED!\n"
                     "**************************************\n")
        sys.exit(1)
    else:
        emblog.debug("\nLinting PASSED.\n")


if __name__ == '__main__':
    main()
