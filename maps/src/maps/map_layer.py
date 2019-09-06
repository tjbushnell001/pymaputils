from enum import Enum


class MapType(Enum):
    LANE = "LANE"
    LANE_ANNOTATION = "LANE_ANNOTATION"
    ROAD = "ROAD"
    MAP_READER = "MAP_READER"
    FREE_SPACE = "FREE_SPACE"
    RADAR_ZONE = "RADAR_ZONE"
    LOCALIZATION_ZONE = "LOCALIZATION_ZONE"
    DISENGAGE_ZONE = "DISENGAGE_ZONE"


class MapLayer(object):
    def __init__(self, layer_type, layer_name=''):
        self.layer_type = layer_type
        self.layer_name = layer_name

    # TODO: Open Questions:
    #  Do we want the generic get_feature / get_features method here? This would essentially become SingleMapLayer
    #  Restrict all map layers to be geojson user facing?
