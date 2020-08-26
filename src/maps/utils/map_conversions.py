"""Utility Python library for map related operations."""

import numpy as np
import utm

def get_utm_zone(lat, lon):
    """ Get the UTM zone for the given a coordinates.

    Returns: (int, bool) tuple indicating utm zone and hemisphere
    """

    utm_zone, zone_letter = utm.from_latlon(lat, lon)[2:]
    northern = zone_letter >= 'N'
    return utm_zone, northern

def convert_vehicle_frame_to_lat_lon(veh_x, veh_y, ego_heading_gcs, ego_lat, ego_lon, utm_zone=None):
    """ Converts a point in vehicle frame to a GPS coordinate, given the GPS coordinates of ego.

    Arguments:
      veh_x: x coordinate in vehicle frame to convert to latitude, longitude
      veh_y: y coordinate in vehicle frame to convert to latitude, longitude
      ego_gcs_heading: heading of ego (in degrees NED, clockwise)
      ego_lat: latitude of ego
      ego_lng: longitude of ego
      utm_zone: (optional) utm zone tuple (per get_utm_zone)

    Returns: (lat, lon) tuple transformed into latitude, longitude
    """

    if utm_zone is None:
        utm_zone = get_utm_zone(ego_lat, ego_lon)

    ego_utm_x, ego_utm_y = convert_lat_lon_to_utm(ego_lat, ego_lon, utm_zone)
    ego_utm_theta = get_utm_theta(utm_zone, ego_heading_gcs, ego_lat, ego_lon)
    utm_x, utm_y, _ = convert_vehicle_frame_to_utm(veh_x, veh_y, ego_utm_x, ego_utm_y, ego_utm_theta)

    return convert_utm_to_lat_lon(utm_x, utm_y, utm_zone)

def convert_lat_lon_to_vehicle_frame(lat, lon, ego_heading_gcs, ego_lat, ego_lon, utm_zone=None):
    """ Converts a GPS coordinate into vehicle frame, given the GPS coordinates of ego.

    Arguments:
      lat: latitude to convert into vehicle frame
      lon: longitude to convert into vehicle frame
      ego_gcs_heading: heading of ego (in degrees NED, clockwise)
      ego_lat: latitude of ego
      ego_lng: longitude of ego
      utm_zone: (optional) utm zone tuple (per get_utm_zone)

    Returns: (veh_x, veh_y) tuple transformed into vehicle frame
    """

    if utm_zone is None:
        utm_zone = get_utm_zone(ego_lat, ego_lon)

    ego_utm_x, ego_utm_y = convert_lat_lon_to_utm(ego_lat, ego_lon, utm_zone)
    ego_utm_theta = get_utm_theta(utm_zone, ego_heading_gcs, ego_lat, ego_lon)
    utm_x, utm_y = convert_lat_lon_to_utm(lat, lon, utm_zone)

    return convert_utm_to_vehicle_frame(utm_x, utm_y, ego_utm_x, ego_utm_y, ego_utm_theta)[:2]

def get_utm_theta(utm_zone, heading_gcs, lat, lon):
    """ GCS heading to UTM theta (in radians, ENU, counter-clockwise) at the given coordinates.

    Arguments:
      utm_zone: utm zone tuple (per get_utm_zone)
      heading_gcs: gcs heading (in degrees NED, clockwise)
      lat: latitude
      lon: longitude

    Returns: utm theta (in radians, ENU, counter-clockwise)
    """

    utm_heading = get_utm_heading(utm_zone, heading_gcs, lat, lon)
    return (np.pi / 2.0) - np.radians(utm_heading)


def get_utm_heading(utm_zone, heading_gcs, lat, lon):
    """ GCS heading to UTM heading (in degrees NED, clockwise) at the given coordinates.

    WARNING: This angle is measured clockwise which is the opposite of utm
    heading. 0 is north. Returns in degrees. For most applications (e.g. trig functions) you
    should probably be using get_utm_theta.

    Arguments:
      utm_zone: utm zone tuple (per get_utm_zone)
      heading_gcs: gcs heading (in degrees NED, clockwise)
      lat: latitude
      lon: longitude

    Returns: utm heading (in degrees NED, clockwise)
    """

    lon_0 = get_utm_zone_central_meridian(utm_zone[0])
    lat_rad = np.radians(lat) # pylint: disable=assignment-from-no-return
    lon_rad = np.radians(lon) # pylint: disable=assignment-from-no-return
    lon_0_rad = np.radians(lon_0) # pylint: disable=assignment-from-no-return
    convergence_rad = np.arctan(np.tan(lon_rad - lon_0_rad) * np.sin(lat_rad)) # pylint: disable=assignment-from-no-return
    return heading_gcs - np.degrees(convergence_rad)


def get_utm_zone_central_meridian(utm_zone):
    """Computes the central meridian of the given UTM zone."""

    return 6.0 * utm_zone - 183


def convert_vehicle_frame_to_utm(veh_x, veh_y, ego_utm_x, ego_utm_y, ego_utm_theta, veh_theta = None):
    """ Converts a point in vehicle frame to UTM, given the utm coordinates of the vehicle.

    Arguments:
      veh_x: x coordinate in vehicle frame to convert to UTM
      veh_y: y coordinate in vehicle frame to convert to UTM
      ego_utm_x: x coordinate of ego in UTM
      ego_utm_y: y coordinate of ego in UTM
      ego_utm_theta: theta of ego in UTM
      veh_theta: (optional) theta in vehicle frame to convert to UTM

    Returns: (utm_x, utm_y, utm_theta) tuple transformed into UTM
    """

    sin_ego_theta = np.sin(ego_utm_theta)
    cos_ego_theta = np.cos(ego_utm_theta)

    utm_x = veh_x * cos_ego_theta - veh_y * sin_ego_theta + ego_utm_x
    utm_y = veh_x * sin_ego_theta + veh_y * cos_ego_theta + ego_utm_y
    utm_theta = None
    if veh_theta is not None:
        utm_theta = veh_theta + ego_utm_theta

    return utm_x, utm_y, utm_theta

def convert_utm_to_vehicle_frame(utm_x, utm_y, ego_utm_x, ego_utm_y, ego_utm_theta, utm_theta = None):
    """ Converts a UTM coordinate into a vehicle frame, given a UTM vehicle pose.

    Arguments:
      utm_x: x UTM coordinate to convert to vehicle frame
      utm_y: y UTM coordinate to convert to vehicle frame
      ego_utm_x: x coordinate of ego in UTM
      ego_utm_y: y coordinate of ego in UTM
      ego_utm_theta: theta of ego in UTM
      utm_theta: (optional) UTM theta to convert to vehicle frame

    Returns: (veh_x, veh_y, veh_theta) tuple transformed into vehicle frame
    """

    sin_ego_theta = np.sin(-ego_utm_theta)
    cos_ego_theta = np.cos(-ego_utm_theta)

    tmp_x = utm_x - ego_utm_x
    tmp_y = utm_y - ego_utm_y

    veh_x = tmp_x * cos_ego_theta - tmp_y * sin_ego_theta
    veh_y = tmp_x * sin_ego_theta + tmp_y * cos_ego_theta
    veh_theta = None
    if utm_theta is not None:
        veh_theta = utm_theta - ego_utm_theta

    return veh_x, veh_y, veh_theta

def convert_lat_lon_to_utm(lat, lon, utm_zone):
    """ Converts a GPS coordinates to UTM coordinate in the given UTM zone.

    Arguments:
      lat: latitude to convert into UTM
      lon: longitude to convert into UTM
      utm_zone: utm zone tuple (per get_utm_zone)

    Returns: (utm_x, utm_y) tuple transformed into UTM
    """

    return utm.from_latlon(lat, lon, force_zone_number=utm_zone[0])[:2]

def convert_utm_to_lat_lon(x, y, utm_zone):
    """ Converts a UTM coordinate to a GPS coordinate.

    Arguments:
      x: x UTM coordinate to convert
      y: y UTM coordinate to convert
      utm_zone: utm zone tuple (per get_utm_zone)

    Returns: (lat, lon) tuple transformed into latitude, longitude
    """

    return utm.to_latlon(x, y, utm_zone[0], northern=utm_zone[1])
