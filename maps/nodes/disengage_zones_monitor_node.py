import json

from applanix_msgs.msg import NavigationSolution
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
from shapely.geometry import Point
from shapely.geometry import Polygon

ERROR = DiagnosticStatus.ERROR
OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN

# TODO: make this not hardcoded
MAP_FOLDER = '/home/embark/catkin_ws/src/brain/perception/lane_map_server/maps/tiled_maps/usa'


class DisengageZonesMonitorNode(object):
    def __init__(self):
        rospy.Subscriber('/applanix/nav_drop', NavigationSolution, self.check_zones, tcp_nodelay=True)

        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1, tcp_nodelay=True)

        try:
            with open(MAP_FOLDER + '/annotations/disengage_zones.json', 'r') as disengage_zones_file:
                disengage_zones_geojson = json.load(disengage_zones_file)

            self.disengage_zones = []

            features = disengage_zones_geojson['features']
            for feature in features:
                assert feature['geometry']['type'] == 'Polygon'

                # assumes one polygon per feature, AND no nested polygons
                # e.g. internal exclusion zone within a polygon is not allowed
                assert len(feature['geometry']['coordinates']) == 1

                long_lat_coords = feature['geometry']['coordinates'][0]
                disengage_zone = Polygon(long_lat_coords)
                self.disengage_zones.append(disengage_zone)
        except:  # if node doesn't initialize, publish error to diagnostics
            self.diagnostics_pub(ERROR, "Node unable to initialize.")

    def check_zones(self, msg):
        long_lat_pt = Point((msg.longitude, msg.latitude))
        status = OK
        status_message = "Not in a disengagement zone based on latest GPS position."

        for zone in self.disengage_zones:
            if zone.contains(long_lat_pt):
                status = ERROR
                status_message = "In a disengagement zone based on latest GPS position."
                break

        self.publish_diagnostics_array(status, status_message)

    def publish_diagnostics_array(self, status, message):
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = rospy.Time.now()
        prefix = 'map_disengage_zone_monitor:'

        in_disengage_zone_status = DiagnosticStatus(
            name=prefix + 'in_disengage_zone_status',
            level=status,
            message=message
        )

        diagnostic_array.status.append(in_disengage_zone_status)
        self.diagnostics_pub.publish(diagnostic_array)


if __name__ == '__main__':
    node_name = 'disengage_zone_monitoring_node'
    rospy.init_node(node_name)
    node = DisengageZonesMonitorNode()

    rospy.spin()
