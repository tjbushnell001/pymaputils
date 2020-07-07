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
    LIDAR_LINE = "LIDAR_LINE"
    LANE_PREFERENCE = "LANE_PREFERENCE"


class LaneAnnotationLayerName(object):
    """
    Defines Lane Annotation Layer names used as filenames in tiled_maps.

    Sample usage:
    rmz = maps.get_layer(MapType.LANE_ANNOTATION,
                         LaneAnnotationLayerName.ROAD_MARKING_ZONES)
    rather than hardcoding string like in:
    rmz = maps.get_layer(MapType.LANE_ANNOTATION, 'road_marking_zones')
    """
    NO_PASSING_ZONES = 'no_passing_zones'
    DISENGAGE_ZONES = 'disengage_zones'
    DISENGAGE_ZONES_LTG = 'disengage_zones_ltg'
    UNDRIVABLE_RIGHT_SHOULDER_ZONES = 'undrivable_right_shoulder_zones'
    ROAD_MARKING_ZONES = 'road_marking_zones'
