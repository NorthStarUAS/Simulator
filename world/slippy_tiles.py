from math import asinh, atan, cos, degrees, pi, radians, sinh, tan
import numpy as np
import os

# Slippy map tile scheme.  Reference:
# https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for code, tips,
# tricks, explanation, math, etc.

eq_m = 40075016.686

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - asinh(tan(lat_rad)) / pi) / 2.0 * n)
    return xtile, ytile

# upper left coordinate
def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = atan(sinh(pi * (1 - 2 * ytile / n)))
    lat_deg = degrees(lat_rad)
    return lat_deg, lon_deg

def get_tile_size(x, y, zoom):
    # return tile size in degrees and (estimated) meters
    lat_nw, lon_nw = num2deg(x, y, zoom)
    lat_cent, lon_cent = num2deg(x+0.5, y+0.5, zoom)
    lat_sw, lon_sw = num2deg(x+1, y+1, zoom)
    dlon = abs(lon_nw - lon_sw)
    dlat = abs(lat_nw - lat_sw)
    dx_m = dlon * cos(radians(lat_cent)) * (eq_m/360)
    dy_m = dlat * (eq_m/360)
    return dlon, dlat, dx_m, dy_m, lat_cent, lon_cent

# def get_path(lat_deg, lon_deg, zoom):
#     x, y = deg2num(lat_deg, lon_deg, zoom)
#     dir = os.path.join( "%d" % zoom, "%d" % x, "%d" % y )
#     return dir

def get_tiles_in_range(lat_deg, lon_deg, zoom, radius_m):
    # fixme: test edge case for wrap around to make sure we aren't generating
    # bogus tile paths!

    # meters per degree
    x_mpd = cos(radians(lat_deg)) * (eq_m/360)
    y_mpd = eq_m/360

    # radius in x and y degrees
    rlon = radius_m / x_mpd
    rlat = radius_m / y_mpd
    print("get_range:", lat_deg, lon_deg, radius_m, rlat, rlon)

    # tile size
    x, y = deg2num(lat_deg, lon_deg, zoom)  # tile
    dx, dy, dx_m, dy_m, clat, clon = get_tile_size(x, y, zoom)  # tile size in deg / m

    tiles = []
    x1, y1 = deg2num(lat_deg + rlat, lon_deg - rlon, zoom)
    x2, y2 = deg2num(lat_deg - rlat, lon_deg + rlon, zoom)
    for x in range(x1, x2+1):
        for y in range(y1, y2+1):
            tile = ( zoom, x, y )
            print(zoom, x, y)
            tiles.append(tile)
    return tiles

# def get_range_paths(lat_deg, lon_deg, zoom, dlat, dlon):
#     # fixme: test edge case for wrap around to make sure we aren't generating
#     # bogus tile paths!

#     x, y = deg2num(lat_deg, lon_deg, zoom)  # tile
#     dx, dy, dx_m, dy_m = get_tile_size(x, y, zoom)  # tile size in deg / m
#     print("get_range:", lat_deg, lon_deg, dlat, dlon)
#     print(" dx, dy:", dx, dy, dx_m, dy_m)
#     paths = []
#     for x in np.arange(lon_deg - 0.5*dlon, lon_deg + 0.5*dlon, dx):
#         for y in np.arange(lat_deg - 0.5*dlat, lat_deg + 0.5*dlat, dy):
#             path = get_path(y, x, zoom)
#             print(x, y, path)
#             paths.append(path)
#     return paths
