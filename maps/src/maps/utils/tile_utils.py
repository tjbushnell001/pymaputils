"""
Library for utility functions related to tile math including id encoding / decoding and lat / long conversions.
"""
import utm


def morton_encode(i, j, tile_level):
    """
    Encode two numbers using morton encoding (see https://en.wikipedia.org/wiki/Z-order_curve)
    :param i: the first number to encode
    :param j: the second number to encode
    :param tile_level: the here tile level
    :return: the morton value
    """
    v = 0
    for l in xrange(tile_level):
        v += (i & (1 << l)) << (l + 1)
        v += (j & (1 << l)) << l
    return v


def morton_decode(morton_val, tile_level):
    """
    Decode a morton encoded value.

    :param morton_val: the morton encoded value
    :param tile_level: the here tile level
    :return: two numbers, i and j used to generate the morton value
    """
    i = j = 0
    remainder = morton_val
    for l, v in iterate_bits(morton_val, 2 * tile_level):
        remainder -= v
        if l % 2 == 0:
            j += v >> l / 2
        else:
            i += v >> (l / 2 + 1)

    remainder -= (4 ** tile_level)
    assert remainder == 0, "{0:b}".format(remainder)
    return i, j


def tile_xy_to_tile_id(xy, tile_level):
    """
    Convert xy coordinates to a tile id. Tile xy coordinates are the horizontal and vertical tile numbers. Thus the
    first tile over antarctica at -90, -180 lat, lng will have an x, y of 0, 0.

    :param xy: coordinates of tile
    :param tile_level: the here tile level
    :return: a morton encoded tile id
    """
    x, y = xy
    morton_id = morton_encode(y, x, tile_level)
    morton_id += 4 ** tile_level
    return morton_id


def tile_id_to_tile_xy(tile_id, tile_level):
    """
    Convert a tile id to it's xy coordinate values.

    :param tile_id: the tile id to decode
    :param tile_level: the here tile level
    :return: decoded x, y coordinates
    """
    tile_y, tile_x = morton_decode(tile_id, tile_level)
    print "tile x: {}, tile y: {}".format(tile_x, tile_y)
    lat_lng_to_tile_xy(33.748380,-116.266990, 14)
    return tile_x, tile_y


def iterate_bits(v, num_bits):
    """
    Iterate each bit of a number and yield it's bit placement and the bit number itself.

    :param v: the raw value
    :param num_bits: number of bits to iterate through
    :return: yield a bit's placement and the number itself
    """
    for l in xrange(num_bits):
        yield (l, v & (1 << l))


def lat_lng_to_tile_xy(lat, lng, tile_level):
    """
    Convert a lat lng to tile x, y coordinates

    :param lat: the latitude
    :param lng: the longitude
    :param tile_level: the here tile level
    :return: the x, y coordinate of the tile
    """
    abs_lng_offset = lng + 180.0
    abs_lat_offset = lat + 90.0

    assert abs_lng_offset >= 0.
    assert abs_lat_offset >= 0.

    degrees_per_tile = 360.0 / (1 << tile_level)

    print "abs_lng:", abs_lng_offset, "abs_lat:", abs_lat_offset
    print "degrees_per_tile:", degrees_per_tile

    tile_x = int(abs_lng_offset / degrees_per_tile)
    tile_y = int(abs_lat_offset / degrees_per_tile)

    return tile_x, tile_y


def lat_lng_to_tile_id(lat, lng, tile_level):
    """
    Convert lat lng value to tile id by converting to x, y and then morton encoding

    :param lat: the latitude value
    :param lng: the longitude value
    :param tile_level: the here tile level
    :return: a here tile id encoding the coordinates
    """
    tile_xy = lat_lng_to_tile_xy(lat, lng, tile_level)
    return tile_xy_to_tile_id(tile_xy, tile_level)


def tile_xy_bounds(tile_xy, tile_level):
    """
    Given a tile x, y coordinate, give the bounds of the tile in lat, lng.

    :param tile_xy:
    :param tile_level:
    :return: return the bounds of the tile at coordinates x, y in lat, lng.
    """
    tile_x, tile_y = tile_xy

    degrees_per_tile = 360.0 / (1 << tile_level)
    min_lng = tile_x * degrees_per_tile - 180.0
    min_lat = tile_y * degrees_per_tile - 90.0

    assert 180. >= min_lng >= -180., min_lng
    assert 90. >= min_lat >= -90., min_lat

    return min_lat, min_lng, min_lat + degrees_per_tile, min_lng + degrees_per_tile


def tile_bounds(tile_id, tile_level):
    """
    Return the bounds of tile with id tile_id

    :param tile_id: the id of the tile
    :param tile_level: the here tile level used for this map layer
    :return: the tile bounds as [min_lat, min_lng, max_lat, max_lng]
    """
    tile_xy = tile_id_to_tile_xy(tile_id, tile_level)
    return tile_xy_bounds(tile_xy, tile_level)


def tile_xy_utm_zone(tile_xy, tile_level):
    """
    Given a tile x, y coordinate, compute the utm zone and utm lat band

    :param tile_xy: the x, y coordinates
    :param tile_level: the here tile level used for this map layer
    :return: the utm zone and utm lat band used for this tile
    """
    min_lat, min_lng, max_lat, max_lng = tile_xy_bounds(tile_xy, tile_level)
    center_lat, center_lng = (min_lat + max_lat) / 2, (min_lng + max_lng) / 2
    utm_zone, utm_lat_band = utm.from_latlon(center_lat, center_lng)[2:]

    return utm_zone, utm_lat_band


def tile_utm_zone(tile_id, tile_level):
    """
    Compute the utm zone and utm lat band most accurate for this tile.

    :param tile_id: the id of the tile
    :param tile_level: the here tile level used on this map layer
    :return: the utm zone and utm lat band used for this tile
    """
    tile_xy = tile_id_to_tile_xy(tile_id, tile_level)

    return tile_xy_utm_zone(tile_xy, tile_level)


def norm_tile_xy(tile_xy, tile_level):
    """
    Given a tile x, y coordinate and tile level, compute the x, y coordinates modulo the max tiles to account
    for crossing poles.

    :param tile_xy: the raw tile x, y coordinate
    :param tile_level: the here tile level used on this map layer
    :return: a new tile x, y coordinate within the bounds of valid coordinates
    """
    max_lng_tiles = 1 << tile_level
    max_lat_tiles = (1 << tile_level) / 2

    tile_x, tile_y = tile_xy
    if tile_y < 0 or tile_y > max_lat_tiles:
        # crossed a pole
        tile_x = -tile_x
        tile_y = -tile_y

    tile_x = tile_x % max_lng_tiles
    tile_y = tile_y % max_lat_tiles

    return tile_x, tile_y


def adjacent_tile_xys(tile_xy, tile_level, include_self=False):
    """
    Given a tile x, y coordinate, calculate the adjacent tiles taking into a account the bounds of valid tile
    coordinates.

    :param tile_xy: the center tile x, y coordinate
    :param tile_level: the here tile level used on this map layer
    :param include_self: include the base x, y coordinate in the return set.
    :return: yield a tile id adjacent to the base x, y
    """
    tile_x, tile_y = tile_xy
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            tid = norm_tile_xy((tile_x + dx, tile_y + dy),
                               tile_level)

            if not include_self and tid == tile_xy:
                # skip tile_xy
                continue

            yield tid


def adjacent_tile_ids(tile_id, tile_level, include_self=False):
    """
    Given a tile id, compute the adjacent tile_ids.

    :param tile_id: The base tile to search from
    :param tile_level: the here tile level of this map layer
    :param include_self: whether to include the base tile in the return set
    :return: a list of tiles adjacent to the base tile
    """
    tile_xy = tile_id_to_tile_xy(tile_id, tile_level)
    adjacent_tiles = adjacent_tile_xys(tile_xy, tile_level, include_self=include_self)
    return [tile_xy_to_tile_id(txy, tile_level) for txy in adjacent_tiles]


def sub_tile_xys(tile_xy, tile_level, target_tile_level):
    """
    Given a tile level larger than another tile level, compute all tile x, y coordinates that are within
    the tile defined by tile_xy in the larger tile set (remember that larger tile levels mean smaller tiles).

    :param tile_xy: the base tile to search within in the base tile_level size
    :param tile_level: the size of the base tile
    :param target_tile_level: the new tile level to search for tiles
    :return: yield a list of tile id's from the new set that are within the base tile
    """
    assert target_tile_level >= tile_level

    tile_factor = 1 << (target_tile_level - tile_level)

    tile_x, tile_y = tile_xy
    for subx in xrange(tile_factor):
        for suby in xrange(tile_factor):
            tid = (tile_x * tile_factor + subx,
                   tile_y * tile_factor + suby)
            yield tid


def sub_tile_ids(tile_id, tile_level, target_tile_level):
    """
    Given a tile level smaller than another tile level, compute all tile ids coordinates that are within
    the tile defined by tile_xy in the larger tile set (remember that smaller tile levels mean larger tiles).

    :param tile_id: the tile id to search within
    :param tile_level: the size of the base tile
    :param target_tile_level: the new tile level to search for tiles
    :return: a list of tile id's from the new set that are within the base tile
    """
    tile_xy = tile_id_to_tile_xy(tile_id, tile_level)
    target_tile_xys = sub_tile_xys(tile_xy, tile_level, target_tile_level)

    return [tile_xy_to_tile_id(tid, target_tile_level) for tid in target_tile_xys]


def super_tile_xy(tile_xy, tile_level, target_tile_level):
    """
    Given a tile level larger than another tile level, compute the tile x, y coordinate of the larger parent
    tile in the smaller tile level (remember that smaller tile levels mean larger tiles).

    :param tile_xy: the tile x, y coordinate to search from
    :param tile_level: the old tile level
    :param target_tile_level: the new tile level
    :return: an x, y coordinate of the larger parent tile
    """
    assert target_tile_level <= tile_level

    tile_factor = 1 << (tile_level - target_tile_level)

    tile_x = int(tile_xy[0] / tile_factor)
    tile_y = int(tile_xy[1] / tile_factor)

    return tile_x, tile_y


def super_tile_id(tile_id, tile_level, target_tile_level):
    """
    Given a tile level larger than another tile level and a tile id, compute the tile id of the larger parent
    tile in the smaller tile level (remember that smaller tile levels mean larger tiles).

    :param tile_id: the tile id of the smaller sub tile
    :param tile_level: the old tile level
    :param target_tile_level: the new tile level
    :return: a tile id of the larger parent tile in the new target_tile_level
    """
    tile_xy = tile_id_to_tile_xy(tile_id, tile_level)
    target_tile_xy = super_tile_xy(tile_xy, tile_level, target_tile_level)

    return tile_xy_to_tile_id(target_tile_xy, target_tile_level)
