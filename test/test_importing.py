import unittest

import setup_env  # pylint: disable=unused-import


# These tests simply check if imports work, so unused-variable is expected.
# pylint: disable=unused-variable
class TestImporting(unittest.TestCase):
    def test_maps_importing(self):
        import maps.feature_dict
        import maps.geojson_tiled_map
        import maps.issue_types
        import maps.issues
        import maps.lane_maps
        import maps.map_layers
        import maps.map_types
        import maps.road_graph
        import maps.routing
        import maps.tiled_map_layer

    def test_maps_linting_importing(self):
        import maps.linting.junction_linter
        import maps.linting.lane_linter
        import maps.linting.route_linter

    def test_maps_utils_importing(self):
        import maps.utils.emblog
        import maps.utils.geojson_utils
        import maps.utils.lane_map_utils
        import maps.utils.lane_utils
        import maps.utils.lru_cache
        import maps.utils.ref_utils
        import maps.utils.routing_utils
        import maps.utils.tile_linker
        import maps.utils.tile_utils
        import maps.utils.translator

# pylint: enable=unused-variable
