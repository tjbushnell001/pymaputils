import geojson
import shapely
import shapely.geometry
import shapely.validation
import os
import utm

from collections import OrderedDict


class HashableDict(dict):
    """ Add a hashing function for dicts. """
    def __hash__(self):
        return hash(tuple(sorted(self.values())))


def hashify(obj):
    """
    Recursively convert all dicts to HashableDict. Does a deep search into all lists and searches for nested dicts.

    :param obj: the object to be hashified
    :return: the updated object
    """
    if isinstance(obj, HashableDict):
        return obj
    elif isinstance(obj, list):
        return [hashify(i) for i in obj]
    elif isinstance(obj, dict):
        return HashableDict({key: hashify(value) for key, value in obj.iteritems()})
    return obj


class FeatureDict(object):
    """
    Our custom representation of a geojson FeatureCollection. All of our geojson features have "ref" objects that we
    parse and add a hash function to.
    """
    def __init__(self, collection):
        self.collection = collection
        assert collection is None or isinstance(collection, geojson.FeatureCollection)
        self.features = {}
        for f in collection.features:
            if f.feature_type not in self.features:
                self.features[f.feature_type] = OrderedDict()
            f.ref = hashify(f.ref)
            f.properties = hashify(f.properties)
            self.features[f.feature_type][f.ref] = f

    def get_feature(self, ref):
        return self.get_features(ref['type'].replace('_ref', '')).get(ref)

    def get_features(self, feature_type):
        return self.features.get(feature_type, {})

    def all_features(self):
        return self.features.values()

    def add_feature(self, feature):
        if feature.feature_type not in self.features:
            self.features[feature.feature_type] = OrderedDict()

        assert feature.ref not in self.features[feature.feature_type]

        self.features[feature.feature_type][feature.ref] = feature
        self.collection.features.append(feature)

    def write(self, file_path, msg=None):
        """
        Write this feature collection to a file.

        :param file_path:
        :param msg: An optional confirmation message
        :return:
        """
        with open(file_path, 'w') as f:
            geojson.dump(self.collection, f, sort_keys=True, separators=(',', ':'), indent=1)
            if msg:
                print(msg)

    # --------------
    # Properties
    # --------------

    @property
    def id(self):
        return self.collection.get('id', None)


def get_feature_type(ref):
    """
    Given a ref, return the type of feature that this refers to.

    :param ref: a ref id
    :return: a string of feature_type
    """
    return ref['type'].replace('_ref', '')


def create_feature(feature_type, feature_ref, geometry, **properties):
    """
    Create a geojson Feature object with the given fields.

    :param feature_type: the type of feature being created. Can be any generic string, but should be one of the
                         standard feature_types (e.g. lane_group, lane, lane_boundary, etc)
    :param feature_ref: a ref dict
    :param geometry: a geojson geometry (or shapely geometry)
    :param properties: any extra keyword arg will be converted to a property of the new geojson feature
    :return: new geojson Feature object
    """
    f = geojson.Feature(geometry=geometry,
                        feature_type=feature_type,
                        ref=feature_ref,
                        properties=properties)

    return f


def create_feature_collection(feature_type, feature_id, features, **properties):
    """
    Create a geojson FeatureCollection object with the given fields.

    :param feature_type: the type describing the entire collection (e.g. route, tile, road_tile, etc)
    :param feature_id: a unique id to set mark the feature
    :param features: a list of geojson Features to add to the collection
    :param properties: any extra keyword arg will be converted to a property of the new geojson feature collection
    :return: new geojson FeatureCollection object
    """
    fc = geojson.FeatureCollection(features,
                                   id=feature_id,
                                   feature_type=feature_type,
                                   properties=properties)
    return fc


def downsample_line(points, utm_zone, utm_lat_band, tolerance=1.0, min_points=20):
    """
    Reduces the number of (downsamples) points in a set of points using shapely's simplify method. Converts the lat lon
    points to utm to do the downsampling and converts back to lat lon for the return value. This ensures we can use
    meters as our tolerance value. Note however that if your points cover a range on the order of hundreds of miles
    of lat lon, there will be significant distortion from the utm conversion.

    :param points: the set of points to downsample
    :param utm_zone: the utm zone to use as the conversion zone
    :param utm_lat_band: the utm lat ban to use for utm conversion
    :param tolerance: the tolerance to use for the downsample (max distance of distortion allowed for points)
    :param min_points: the minimum number of points in the return
    :return: list of simplified coordinate points in lat lon
    """
    points = list(points)

    if len(points) < min_points:
        return points

    # force the utm_zone in case we cross utm boundaries
    utm_points = (utm.from_latlon(p[1], p[0], force_zone_number=utm_zone)[:2]
                  for p in points)

    line = shapely.geometry.LineString(utm_points)
    line = line.simplify(tolerance, preserve_topology=False)

    rev = lambda (x, y): (y, x)

    ll_points = [rev(utm.to_latlon(p[0], p[1], utm_zone, utm_lat_band)[:2])
                 for p in line.coords]

    return ll_points


# TODO: create poly to boundaries
def boundaries_to_poly(left_boundary, right_boundary):
    """
    Takes two line boundaries and converts them into a polygon. This is done by reversing the right boundary,
    combining the two boundaries into a single set, and looping back to include the first point again, forming
    a valid geojson polygon.
    :param left_boundary: an iterable of points defining the left lane line boundary
    :param right_boundary: an iterable of points defining the right lane line boundary
    :return: a shapely Polygon
    """
    poly_points = list(left_boundary)
    poly_points.extend(reversed(right_boundary))
    poly_points.append(poly_points[0])

    poly = shapely.geometry.Polygon(poly_points)
    # TODO: these can be invalid with invalid objects. is that different from geojson.is_valid?
    # if not poly.is_valid:
    #    poly = poly.buffer(0)
    # assert poly.is_valid, shapely.validation.explain_validity(poly)

    return poly


def bbox_to_poly(min_lat, min_lng, max_lat, max_lng):
    """
    Converts a set of rectangular boundary dimensions in lat lon into a shapely polygon.

    :param min_lat: minimum latitude value
    :param min_lng: minimum longitude value
    :param max_lat: maximum latitude value
    :param max_lng: maximum longitude value
    :return: a shapely polygon constructed with the four corners defined by the boundary dimensions.
    """
    poly = shapely.geometry.Polygon([
        (min_lng, min_lat),
        (min_lng, max_lat),
        (max_lng, max_lat),
        (max_lng, min_lat),
        (min_lng, min_lat)])
    # last point must be repeated in a linear ring
    return poly


def lane_group_ref_from_lane_ref(lane_ref):
    """
    Convert a lane ref into it's parent lane group ref

    :param lane_ref: the full lane.ref of the lane
    :return: a hashable ref key for the lane group
    """
    return hashify({'id': lane_ref['lane_group_id'], 'tile_id': lane_ref['tile_id'], 'type': 'lane_group_ref'})


def load_geojson_collection(file_path):
    """
    Load a geojson file adding our custom ref parsing logic.

    :param file_path: the full file path of the file to load
    :return: a FeatureDict object with fully index-able refs
    """
    if not os.path.exists(file_path):
        return None
    with open(file_path, 'r') as f:
        return FeatureDict(geojson.load(f))


def write_geojson_tile(tile_id, prefix, feature_collection):
    """
    Write a geojson feature_collection to disk as a json file.

    :param tile_id: the id of the tile being written
    :param prefix: the file path to write the geojson file to
    :param feature_collection: the feature collection object
    :return: None
    """
    fn = os.path.join(prefix, '{}.json'.format(tile_id))

    geojson.dump(feature_collection, open(fn, 'w'),
                 sort_keys=True, separators=(',', ':'), indent=0)


def connector_ref_from_junction_ref(junction_ref):
    """
    Return the connector ref from a junction ref.

    :param junction_ref: the ref object of the junction within a connector
    :return:
    """
    return hashify({'id': junction_ref['connector_id'], 'tile_id': junction_ref['tile_id'], 'type': 'connector_ref'})
