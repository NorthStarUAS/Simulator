import numpy as np
import os
import pathlib
import pickle
import scipy.interpolate
import scipy.spatial
import sys
import time

from panda3d.core import *

import navpy

sys.path.append("../..")
from comms import comms_mgr

from ..world import tile_cache
from ..world import slippy_tiles

# https://github.com/tilezen/joerd/tree/master/docs
# https://s3.amazonaws.com/elevation-tiles-prod/{terrarium,normal}/8/62/90.png

dot_root = ".scenery_viewer"
url = "s3.amazonaws.com"
root = "/elevation-tiles-prod/terrarium"
terrain_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "terrain")
terra_cache = tile_cache.SlippyCache(terrain_dir, url, root)

osm_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "osm")
osm_cache = tile_cache.SlippyCache(osm_dir, "tile.openstreetmap.org", "")

# https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key=UN7fFi8RQaOjB7HlKzXc
sat_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "satellite")
sat_cache = tile_cache.SlippyCache(sat_dir, "api.maptiler.com", "/tiles/satellite-v2", ext=".jpg", options="?key=UN7fFi8RQaOjB7HlKzXc")

tin_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "tin")
tin_cache = tile_cache.SlippyCache(tin_dir, "", "", ext=".tin")

sat_mat = Material()
sat_mat.setAmbient((1, 1, 1, 1.0))
sat_mat.setDiffuse((1, 1, 1, 1.0))

poly_mat = Material()
poly_mat.setAmbient((0.3, 0.8, 0.3, 0.6))
poly_mat.setDiffuse((0.3, 0.8, 0.3, 0.6))

terra_format = GeomVertexFormat.get_v3n3t2()

#min_error = 25
quick = False
if quick:
    max_fit_pts_at_zoom_10 = 10
else:
    max_fit_pts_at_zoom_10 = 6400

tin_version = 4

def optimize_1d(rem_pts, rem_vals, min_error, axis):
    # print(rem_pts, rem_vals)
    # assume edge_pts ends are first and last in the array
    fit_pts = [ rem_pts[0] ]
    fit_x = [ rem_pts[0][axis] ]
    fit_y = [ rem_vals[0] ]
    fit_pts.append(rem_pts[-1])
    fit_x.append(rem_pts[-1][axis])
    fit_y.append(rem_vals[-1])
    # print(fit_x, fit_y)
    del(rem_pts[-1])
    del(rem_vals[-1])
    del(rem_pts[0])
    del(rem_vals[0])
    done = False
    current_min = 9999
    while not done:
        interp = scipy.interpolate.interp1d(fit_x, fit_y)
        pts = np.array(rem_pts).T[axis]
        # print("pts:", pts)
        # print("rem:", pts[0], pts[1])
        vals = interp(pts)
        # print("vals:", vals)
        # print("remain_pts.T:", np.array(remain_pts).T)
        errors = np.abs(np.array(rem_vals) - vals)
        # print("errors:", errors)
        i = np.argmax(errors)
        # print("max error index:", i, "value:", errors[i])
        if errors[i] < min_error:
            done = True
        else:
            fit_pts.append( rem_pts[i] )
            fit_x.append( rem_pts[i][axis] )
            fit_y.append( rem_vals[i] )
            del(rem_pts[i])
            del(rem_vals[i])
        if errors[i] < current_min:
            current_min = errors[i]
            # print(len(fit_x), current_min, "->", min_error)
    if False:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(np.array(rem_pts).T[axis], np.array(rem_vals), "*", color="blue")
        plt.plot(np.array(fit_x), np.array(fit_y), "*", color="red")
        plt.show()
    return fit_pts, fit_y, rem_pts, rem_vals

# extract the altitude values and return the average
def avg_altitude(pts):
    sum = 0
    for (red, green, blue, alpha) in pts:
        sum += (red * 256 + green + blue / 256) - 32768
    return sum / len(pts)

# get corner points and edges with elevations averaged from all neighbor tiles that touch
def get_boundary_vals(zoom_level, x, y):
    print("get boundaries...")

    # step = 2 ** (zoom_level - 10)
    # if step < 1:
    #     step = 1

    print("load tiles...")
    center = terra_cache.get_tile_as_pnm(zoom_level, x, y)
    nw = terra_cache.get_tile_as_pnm(zoom_level, x-1, y-1)
    upper = terra_cache.get_tile_as_pnm(zoom_level, x, y-1)
    ne = terra_cache.get_tile_as_pnm(zoom_level, x+1, y-1)
    left = terra_cache.get_tile_as_pnm(zoom_level, x-1, y)
    right = terra_cache.get_tile_as_pnm(zoom_level, x+1, y)
    sw = terra_cache.get_tile_as_pnm(zoom_level, x-1, y+1)
    bottom = terra_cache.get_tile_as_pnm(zoom_level, x, y+1)
    se = terra_cache.get_tile_as_pnm(zoom_level, x+1, y+1)

    # coordinates
    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)
    cols = center.getXSize()
    rows = center.getYSize()

    # corners
    nw_alt = avg_altitude( [ nw.getPixel(cols-1, rows-1), upper.getPixel(0, rows-1), left.getPixel(cols-1, 0), center.getPixel(0, 0), ])
    ne_alt = avg_altitude( [ upper.getPixel(cols-1, rows-1), ne.getPixel(0, rows-1), center.getPixel(cols-1, 0), right.getPixel(0, 0), ])
    sw_alt = avg_altitude( [ left.getPixel(cols-1, rows-1), center.getPixel(0, rows-1), sw.getPixel(cols-1, 0), bottom.getPixel(0, 0), ])
    se_alt = avg_altitude( [ center.getPixel(cols-1, rows-1), right.getPixel(0, rows-1), bottom.getPixel(cols-1, 0), se.getPixel(0, 0), ])
    # corner_pts = [ [nw_lon, nw_lat], [se_lon, nw_lat], [nw_lon, se_lat], [se_lon, se_lat] ]
    corner_vals = [ nw_alt, ne_alt, sw_alt, se_alt ]
    print("corners:", corner_vals)

    # left edge
    left_vals = []
    for i in range(rows):
        left_vals.append( avg_altitude( [left.getPixel(cols-1, i), center.getPixel(0, i)]) )

    # right edge
    right_vals = []
    for i in range(rows):
        right_vals.append( avg_altitude( [center.getPixel(cols-1, i), right.getPixel(0, i)]) )

    # top edge
    top_vals = []
    for i in range(cols):
        top_vals.append( avg_altitude( [upper.getPixel(i, rows-1), center.getPixel(i, 0)]) )

    # bottome edge
    bottom_vals = []
    for i in range(cols):
        bottom_vals.append( avg_altitude( [center.getPixel(i, rows-1), bottom.getPixel(i, 0)]) )

    # return just the neighbor-averaged heights ... lon/lat are computed at the calling layer
    return corner_vals, left_vals, right_vals, top_vals, bottom_vals

# get corner points and edges with elevations averaged from all neighbor tiles that touch
def gen_height_array_with_neighbors(zoom_level, x, y):
    print("get boundaries...")

    # step = 2 ** (zoom_level - 10)
    # if step < 1:
    #     step = 1

    print("load tiles...")
    center = terra_cache.get_tile_as_pnm(zoom_level, x, y)
    nw = terra_cache.get_tile_as_pnm(zoom_level, x-1, y-1)
    upper = terra_cache.get_tile_as_pnm(zoom_level, x, y-1)
    ne = terra_cache.get_tile_as_pnm(zoom_level, x+1, y-1)
    left = terra_cache.get_tile_as_pnm(zoom_level, x-1, y)
    right = terra_cache.get_tile_as_pnm(zoom_level, x+1, y)
    sw = terra_cache.get_tile_as_pnm(zoom_level, x-1, y+1)
    bottom = terra_cache.get_tile_as_pnm(zoom_level, x, y+1)
    se = terra_cache.get_tile_as_pnm(zoom_level, x+1, y+1)

    # coordinates
    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)
    cols = center.getXSize()
    rows = center.getYSize()

    height = np.array([rows, cols])

    for c in range(cols):
        for r in range(rows):
            # corners
            if r == 0 and c == 0:
                # nw corner
                alt = avg_altitude( [ nw.getPixel(cols-1, rows-1), upper.getPixel(0, rows-1), left.getPixel(cols-1, 0), center.getPixel(0, 0), ])
            elif r == 0 and c == cols-1:
                # ne corner
                alt = avg_altitude( [ upper.getPixel(cols-1, rows-1), ne.getPixel(0, rows-1), center.getPixel(cols-1, 0), right.getPixel(0, 0), ])
            elif r == rows-1 and c == 0:
                # se corner
                alt = avg_altitude( [ left.getPixel(cols-1, rows-1), center.getPixel(0, rows-1), sw.getPixel(cols-1, 0), bottom.getPixel(0, 0), ])
            elif r == rows-1 and c == cols-1:
                # sw corner
                alt = avg_altitude( [ center.getPixel(cols-1, rows-1), right.getPixel(0, rows-1), bottom.getPixel(cols-1, 0), se.getPixel(0, 0), ])
            elif c == 0:
                # left edge
                alt = avg_altitude( [left.getPixel(cols-1, r), center.getPixel(0, r)])
            elif c == cols-1:
                # right edge
                alt = avg_altitude( [center.getPixel(cols-1, r), right.getPixel(0, r)])
            elif r == 0:
                # top edge
                alt = avg_altitude( [upper.getPixel(c, rows-1), center.getPixel(c, 0)])
            elif r == rows-1:
                # bottome edge
                alt = avg_altitude( [center.getPixel(c, rows-1), bottom.getPixel(c, 0)])
            else:
                alt = avg_altitude( [center.getPixel(c, r)])
            height[r,c] = alt

    return height

def fit_tin_new(zoom_level, x, y):
    cols = 0
    rows = 0

    heights = gen_height_array_with_neighbors(zoom_level, x, y)
    (rows, cols) = heights.shape

    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)
    _, _, dx_m, dy_m, center_lat, center_lon = slippy_tiles.get_tile_size(x, y, zoom_level)
    step = 1

    min_error = ((dx_m + dy_m) * 0.5) / 128 # 2 pixel height error at visually optimal distance (for 256x256 textures)
    if min_error < 15:
        min_error = 15
    # step = 2 ** (zoom_level - 10)
    # if step < 1:
    #     step = 1
    if zoom_level > 10:
        max_fit_pts = max_fit_pts_at_zoom_10 / 2**(zoom_level-10)
    else:
        max_fit_pts = max_fit_pts_at_zoom_10
    if max_fit_pts < 4:
        max_fit_pts = 4
    dlat = (nw_lat - se_lat) / rows
    dlon = (se_lon - nw_lon) / cols
    fit_pts = []
    fit_vals = []
    top_pts = []
    top_vals = []
    bottom_pts = []
    bottom_vals = []
    left_pts = []
    left_vals = []
    right_pts = []
    right_vals = []
    remain_pts = []
    remain_vals = []
    for r in range(0, rows, step):
        if r == 0:
            lat = nw_lat
        elif r == rows - step:
            lat = se_lat
        else:
            lat = (nw_lat - 0.5*(dlat*step)) - r*dlat
        for c in range(0, cols, step):
            #print(r, c)
            if c == 0:
                lon = nw_lon
            elif c == cols - step:
                lon = se_lon
            else:
                lon = nw_lon + 0.5*(dlon*step) + c*dlon
            (red, green, blue, alpha) = pnm.getPixel(c, r)
            alt = (red * 256 + green + blue / 256) - 32768
            #print(c, r, (red, green, blue), alt)
            #print(lat, lon, alt)
            #ned = navpy.lla2ned(lat, lon, alt, center_lat, center_lon, 0.0)
            if (r == 0 or r == rows - step) and (c == 0 or c == cols - step):
                fit_pts.append( [lon, lat] )
                # fit_vals.append( alt )

            if r == 0:
                top_pts.append( [lon, lat] )
                # top_vals.append( alt )
            elif r == rows - step:
                bottom_pts.append( [lon, lat] )
                # bottom_vals.append( alt )

            if c == 0:
                left_pts.append( [lon, lat] )
                # left_vals.append( alt )
            elif c == cols - step:
                right_pts.append( [lon, lat] )
                # right_vals.append( alt )

            if not (r == 0 or r == rows - step or c == 0 or c == cols - step):
                # interior
                remain_pts.append([lon, lat] )
                remain_vals.append( alt )
    print("fit:", fit_pts, fit_vals)
    fit_vals, left_vals, right_vals, top_vals, bottom_vals = get_boundary_vals(zoom_level, x, y)

    # surface approximation/optimization
    print("optimizing edges...")
    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(top_pts, top_vals, min_error, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(bottom_pts, bottom_vals, min_error, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(left_pts, left_vals, min_error, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(right_pts, right_vals, min_error, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    print("optimizing interior points ...")
    current_min = 9999
    done = False
    delaunay = scipy.spatial.Delaunay(fit_pts, incremental=True, qhull_options="QJ0.001")
    count = len(fit_pts)
    while not done:
        # interp = scipy.interpolate.LinearNDInterpolator(fit_pts, fit_vals)
        interp = scipy.interpolate.LinearNDInterpolator(delaunay, fit_vals)
        pts = np.array(remain_pts).T
        # print("rem:", pts[0], pts[1])
        vals = interp(pts[0], pts[1])
        # print("vals:", vals)
        # print("remain_pts.T:", np.array(remain_pts).T)
        errors = np.abs(np.array(remain_vals) - vals)
        # print("errors:", errors)
        i = np.argmax(errors)
        # print("max error index:", i, "value:", errors[i])
        if errors[i] < min_error or count >= max_fit_pts:
            done = True
        else:
            fit_pts.append( remain_pts[i] )
            fit_vals.append( remain_vals[i] )
            delaunay.add_points( [ remain_pts[i] ] )
            count += 1
            del(remain_pts[i])
            del(remain_vals[i])
        if errors[i] < current_min:
            current_min = errors[i]
            print(count, current_min, "->", min_error)
    if False:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(np.array(remain_pts).T[0], np.array(remain_pts).T[1], "*", color="blue")
        plt.plot(np.array(fit_pts).T[0], np.array(fit_pts).T[1], "*", color="red")
        plt.show()

    # this is a good place to compute texture coordinates for the fit points
    dlat = nw_lat - se_lat
    dlon = se_lon - nw_lon
    texcoords = []
    for p in fit_pts:
        u = (p[0] - nw_lon) / dlon
        v = (p[1] - se_lat) / dlat
        texcoords.append([u,v])

    # the first 4 fit pts are the corners, vals are the elevations, average
    # these to get a tile elevation
    return fit_pts, fit_vals, [center_lat, center_lon, np.mean(fit_vals[:4])], texcoords

def fit_tin(zoom_level, x, y):
    cols = 0
    rows = 0
    count = 0
    while count < 5:
        # five tries, then die?
        pnm = terra_cache.get_tile_as_pnm(zoom_level, x, y)
        cols = pnm.getXSize()
        rows = pnm.getYSize()
        if cols > 0 and rows > 0:
            break
        else:
            file = terra_cache.ensure_tile_in_cache(zoom_level, x, y) # get the path name
            print("Retrying pnm fetch:", file)
            os.remove(file)
            time.sleep(1)

    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)
    _, _, dx_m, dy_m, center_lat, center_lon = slippy_tiles.get_tile_size(x, y, zoom_level)
    step = 1

    min_error = ((dx_m + dy_m) * 0.5) / 128 # 2 pixel height error at visually optimal distance (for 256x256 textures)
    if min_error < 15:
        min_error = 15
    # step = 2 ** (zoom_level - 10)
    # if step < 1:
    #     step = 1
    if zoom_level > 10:
        max_fit_pts = max_fit_pts_at_zoom_10 / 2**(zoom_level-10)
    else:
        max_fit_pts = max_fit_pts_at_zoom_10
    if max_fit_pts < 4:
        max_fit_pts = 4
    dlat = (nw_lat - se_lat) / rows
    dlon = (se_lon - nw_lon) / cols
    fit_pts = []
    fit_vals = []
    top_pts = []
    top_vals = []
    bottom_pts = []
    bottom_vals = []
    left_pts = []
    left_vals = []
    right_pts = []
    right_vals = []
    remain_pts = []
    remain_vals = []
    for r in range(0, rows, step):
        if r == 0:
            lat = nw_lat
        elif r == rows - step:
            lat = se_lat
        else:
            lat = (nw_lat - 0.5*(dlat*step)) - r*dlat
        for c in range(0, cols, step):
            #print(r, c)
            if c == 0:
                lon = nw_lon
            elif c == cols - step:
                lon = se_lon
            else:
                lon = nw_lon + 0.5*(dlon*step) + c*dlon
            (red, green, blue, alpha) = pnm.getPixel(c, r)
            alt = (red * 256 + green + blue / 256) - 32768
            #print(c, r, (red, green, blue), alt)
            #print(lat, lon, alt)
            #ned = navpy.lla2ned(lat, lon, alt, center_lat, center_lon, 0.0)
            if (r == 0 or r == rows - step) and (c == 0 or c == cols - step):
                fit_pts.append( [lon, lat] )
                # fit_vals.append( alt )

            if r == 0:
                top_pts.append( [lon, lat] )
                # top_vals.append( alt )
            elif r == rows - step:
                bottom_pts.append( [lon, lat] )
                # bottom_vals.append( alt )

            if c == 0:
                left_pts.append( [lon, lat] )
                # left_vals.append( alt )
            elif c == cols - step:
                right_pts.append( [lon, lat] )
                # right_vals.append( alt )

            if not (r == 0 or r == rows - step or c == 0 or c == cols - step):
                # interior
                remain_pts.append([lon, lat] )
                remain_vals.append( alt )
    print("fit:", fit_pts, fit_vals)
    fit_vals, left_vals, right_vals, top_vals, bottom_vals = get_boundary_vals(zoom_level, x, y)

    # surface approximation/optimization
    print("optimizing edges...")
    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(top_pts, top_vals, min_error, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(bottom_pts, bottom_vals, min_error, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(left_pts, left_vals, min_error, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(right_pts, right_vals, min_error, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    print("optimizing interior points ...")
    current_min = 9999
    done = False
    delaunay = scipy.spatial.Delaunay(fit_pts, incremental=True, qhull_options="QJ0.001")
    count = len(fit_pts)
    while not done:
        # interp = scipy.interpolate.LinearNDInterpolator(fit_pts, fit_vals)
        interp = scipy.interpolate.LinearNDInterpolator(delaunay, fit_vals)
        pts = np.array(remain_pts).T
        # print("rem:", pts[0], pts[1])
        vals = interp(pts[0], pts[1])
        # print("vals:", vals)
        # print("remain_pts.T:", np.array(remain_pts).T)
        errors = np.abs(np.array(remain_vals) - vals)
        # print("errors:", errors)
        i = np.argmax(errors)
        # print("max error index:", i, "value:", errors[i])
        if errors[i] < min_error or count >= max_fit_pts:
            done = True
        else:
            fit_pts.append( remain_pts[i] )
            fit_vals.append( remain_vals[i] )
            delaunay.add_points( [ remain_pts[i] ] )
            count += 1
            del(remain_pts[i])
            del(remain_vals[i])
        if errors[i] < current_min:
            current_min = errors[i]
            print(count, current_min, "->", min_error)
    if False:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(np.array(remain_pts).T[0], np.array(remain_pts).T[1], "*", color="blue")
        plt.plot(np.array(fit_pts).T[0], np.array(fit_pts).T[1], "*", color="red")
        plt.show()

    # this is a good place to compute texture coordinates for the fit points
    dlat = nw_lat - se_lat
    dlon = se_lon - nw_lon
    texcoords = []
    for p in fit_pts:
        u = (p[0] - nw_lon) / dlon
        v = (p[1] - se_lat) / dlat
        texcoords.append([u,v])

    # the first 4 fit pts are the corners, vals are the elevations, average
    # these to get a tile elevation
    return fit_pts, fit_vals, [center_lat, center_lon, np.mean(fit_vals[:4])], texcoords

def fit_tin_incremental_but_no_faster(zoom_level, x, y):
    # recognizing that when a new point is inserted into the tin ... only the
    # new triangles connected to the new point change, so we can avoid an
    # increasing n x n comparison situation by only interpolating the locally
    # changed triangle area.  Points outside the region will have a nan
    # interpolation value so we don't need to update those errors.

    pnm = terra_cache.get_tile_as_pnm(zoom_level, x, y)
    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)

    center_lat = (nw_lat + se_lat) * 0.5
    center_lon = (nw_lon + se_lon) * 0.5
    cols = pnm.getXSize()
    rows = pnm.getYSize()
    step = 1
    # step = 2 ** (zoom_level - 10)
    # if step < 1:
    #     step = 1
    if zoom_level > 10:
        max_fit_pts = max_fit_pts_at_zoom_10 / 2**(zoom_level-10)
    else:
        max_fit_pts = max_fit_pts_at_zoom_10
    if max_fit_pts < 4:
        max_fit_pts = 4
    dlat = (nw_lat - se_lat) / rows
    dlon = (se_lon - nw_lon) / cols
    fit_pts = []
    fit_vals = []
    top_pts = []
    top_vals = []
    bottom_pts = []
    bottom_vals = []
    left_pts = []
    left_vals = []
    right_pts = []
    right_vals = []
    remain_pts = []
    remain_vals = []
    for r in range(0, rows, step):
        if r == 0:
            lat = nw_lat
        elif r == rows - step:
            lat = se_lat
        else:
            lat = (nw_lat - 0.5*(dlat*step)) - r*dlat
        for c in range(0, cols, step):
            #print(r, c)
            if c == 0:
                lon = nw_lon
            elif c == cols - step:
                lon = se_lon
            else:
                lon = nw_lon + 0.5*(dlon*step) + c*dlon
            (red, green, blue, alpha) = pnm.getPixel(c, r)
            alt = (red * 256 + green + blue / 256) - 32768
            #print(c, r, (red, green, blue), alt)
            #print(lat, lon, alt)
            #ned = navpy.lla2ned(lat, lon, alt, center_lat, center_lon, 0.0)
            if (r == 0 or r == rows - step) and (c == 0 or c == cols - step):
                fit_pts.append( [lon, lat] )
                # fit_vals.append( alt )

            if r == 0:
                top_pts.append( [lon, lat] )
                # top_vals.append( alt )
            elif r == rows - step:
                bottom_pts.append( [lon, lat] )
                # bottom_vals.append( alt )

            if c == 0:
                left_pts.append( [lon, lat] )
                # left_vals.append( alt )
            elif c == cols - step:
                right_pts.append( [lon, lat] )
                # right_vals.append( alt )

            if not (r == 0 or r == rows - step or c == 0 or c == cols - step):
                # interior
                remain_pts.append([lon, lat] )
                remain_vals.append( alt )
    print("fit:", fit_pts, fit_vals)
    fit_vals, left_vals, right_vals, top_vals, bottom_vals = get_boundary_vals(zoom_level, x, y)

    # surface approximation/optimization
    print("optimizing edges...")
    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(top_pts, top_vals, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(bottom_pts, bottom_vals, axis=0)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(left_pts, left_vals, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)

    incl_pts, incl_vals, rem_pts, rem_vals = optimize_1d(right_pts, right_vals, axis=1)
    fit_pts.extend(incl_pts[1:-1])
    fit_vals.extend(incl_vals[1:-1])
    remain_pts.extend(rem_pts)
    remain_vals.extend(rem_vals)
    remain_errors = np.zeros(len(remain_vals))

    print("optimizing interior points ...")
    current_min = 9999
    done = False
    delaunay = scipy.spatial.Delaunay(fit_pts, incremental=True)
    count = len(fit_pts)
    local_pts = fit_pts
    local_vals = fit_vals
    while not done:
        interp = scipy.interpolate.LinearNDInterpolator(local_pts, local_vals)
        #interp = scipy.interpolate.LinearNDInterpolator(delaunay, fit_vals)
        pts = np.array(remain_pts).T
        # print("rem:", pts[0], pts[1])
        vals = interp(pts[0], pts[1])
        #print("vals:", vals)
        #print("remain_pts.T:", np.array(remain_pts).T)
        errors = np.abs(np.array(remain_vals) - vals)
        # nan_idx = np.isnan(errors)
        good_idx = np.isfinite(errors)
        #print("remain_error:", remain_errors.shape, "errors:", errors.shape)
        remain_errors[good_idx] = errors[good_idx]
        #print("errors:", errors)
        i = np.argmax(remain_errors)
        #print("max error index:", i, "value:", errors[i])
        error_val = remain_errors[i]
        if error_val < min_error or count >= max_fit_pts:
            done = True
        else:
            fit_pts.append( remain_pts[i] )
            fit_vals.append( remain_vals[i] )
            delaunay.add_points( [ remain_pts[i] ] )
            count += 1
            #print("points:", remain_pts[i], delaunay.points.shape, delaunay.points[-1])
            #print("Simplices:", delaunay.find_simplex(remain_pts[i]))
            # print("Simplices (index):", len(delaunay.points), delaunay.vertex_neighbor_vertices[len(delaunay.points)-1])
            # print("Simplices (index):", len(delaunay.points), delaunay.vertex_neighbor_vertices[0].shape, delaunay.vertex_neighbor_vertices[1].shape, delaunay.vertex_neighbor_vertices)
            idx_ptrs, indices = delaunay.vertex_neighbor_vertices
            local_ids = indices[idx_ptrs[len(fit_pts)-1]:idx_ptrs[len(fit_pts)]]
            np.append(local_ids, len(fit_pts)-1)
            local_pts = np.array(fit_pts)[local_ids]
            local_vals = np.array(fit_vals)[local_ids]
            #print("local_pts:", local_pts)
            del(remain_pts[i])
            del(remain_vals[i])
            remain_errors = np.delete(remain_errors, i)
        if error_val < current_min:
            current_min = error_val
            print(count, current_min, "->", min_error)
    if False:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(np.array(remain_pts).T[0], np.array(remain_pts).T[1], "*", color="blue")
        plt.plot(np.array(fit_pts).T[0], np.array(fit_pts).T[1], "*", color="red")
        plt.show()

    # this is a good place to compute texture coordinates for the fit points
    dlat = nw_lat - se_lat
    dlon = se_lon - nw_lon
    texcoords = []
    for p in fit_pts:
        u = (p[0] - nw_lon) / dlon
        v = (p[1] - se_lat) / dlat
        texcoords.append([u,v])

    return fit_pts, fit_vals, center_lat, center_lon, texcoords

def genterrain(zoom_level, x, y, style):
    path = tin_cache.ensure_path_in_cache(zoom_level, x, y)
    tin_file = os.path.join(path, "%d" % y + ".tin")
    print("tin_file:", tin_file)
    if os.path.exists(tin_file):
        with open(tin_file, "rb") as f:
            data = pickle.load(f)
            if type(data) is list:
                version = 0
                fit_pts = data[0]
                fit_vals = data[1]
                center_lat = data[2]
                center_lon = data[3]
                texcoords = data[4]
                version = 0
            else:
                version = data["version"]
                fit_pts = data["fit_pts"]
                fit_vals = data["fit_vals"]
                if version >= 4:
                    center_lla = data["center_lla"]
                texcoords = data["texcoords"]
    if not os.path.exists(tin_file) or version < tin_version:
        fit_pts, fit_vals, center_lla, texcoords = fit_tin(zoom_level, x, y)
        with open(tin_file, "wb") as f:
            data = {"version": tin_version,
                    "fit_pts": fit_pts,
                     "fit_vals": fit_vals,
                     "center_lla": center_lla,
                     "texcoords": texcoords,
                    }
            pickle.dump( data, f)

    if style == "satellite":
        # texture test
        defer_loading = False
        if defer_loading:
            tex_filename = sat_cache.ensure_tile_in_cache(zoom_level, x, y)
            print("texture name:", tex_filename)
            surface_tex = Texture(tex_filename)
            surface_tex.read(tex_filename)
        else:
            #surface_tex = osm_cache.get_tile_as_tex(zoom_level, x, y)
            surface_tex = sat_cache.get_tile_as_tex(zoom_level, x, y)
        surface_tex.setWrapU(Texture.WM_clamp)
        surface_tex.setWrapV(Texture.WM_clamp)
        surface_tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
        surface_tex.setAnisotropicDegree(16)
        if not defer_loading:
            surface_tex.generateRamMipmapImages()

    # compute Delaunay triangulation in lon/lat space
    tris = scipy.spatial.Delaunay(fit_pts)
    print("tris:", len(tris.simplices))
    if False:
        import matplotlib.pyplot as plt
        plt.triplot(points[:,0], points[:,1], tris.simplices)
        plt.plot(points[:,0], points[:,1], 'o')
        plt.show()

    # convert to xyz (via ned)
    points = []
    vals = []
    for i in range(len(fit_pts)):
        ned = navpy.lla2ned(fit_pts[i][1], fit_pts[i][0], fit_vals[i], center_lla[0], center_lla[1], center_lla[2])
        points.append( [ned[1], ned[0]] )
        vals.append(-ned[2])

    # compute face normals
    normals = [[] for x in range(len(points))]
    for t in tris.simplices:
        p1 = np.array([points[t[0]][0], points[t[0]][1], vals[t[0]]])
        p2 = np.array([points[t[1]][0], points[t[1]][1], vals[t[1]]])
        p3 = np.array([points[t[2]][0], points[t[2]][1], vals[t[2]]])
        #print(p1, p2, p3)
        v1 = p2 - p1
        v2 = p3 - p1
        v = np.cross(v1, v2)
        n = v / np.linalg.norm(v)
        #print("normal:", n)
        normals[t[0]].append(n)
        normals[t[1]].append(n)
        normals[t[2]].append(n)
    #print("normals:", normals)

    # average the connected face normals to compute each individual vertex normal
    for i in range(len(normals)):
        sum = np.zeros(3)
        count = 0
        for n in normals[i]:
            sum += n
            count += 1
        avg = sum / count
        avg = avg / np.linalg.norm(avg)
        normals[i] = avg

    print("terrain")
    vdata = GeomVertexData("terrain", terra_format, Geom.UHStatic)
    vdata.setNumRows(len(points))

    vertex = GeomVertexWriter(vdata, "vertex")
    for i in range(len(points)):
        vertex.addData3f(points[i][0], points[i][1], vals[i])

    norm_writer = GeomVertexWriter(vdata, "normal")
    for i in range(len(normals)):
        norm_writer.addData3f(normals[i][0], normals[i][1], normals[i][2])

    tex_writer = GeomVertexWriter(vdata, 'texcoord')
    for i in range(len(texcoords)):
        tex_writer.addData2f(texcoords[i][0], texcoords[i][1])

    prim = GeomTriangles(Geom.UHStatic) # don't expect geometry to change
    for t in tris.simplices:
        prim.addVertices(t[0], t[1], t[2])
    prim.closePrimitive()

    geom = Geom(vdata)
    geom.addPrimitive(prim)

    node = GeomNode("geom")
    node.addGeom(geom)

    tile_node = NodePath( "%d/%d/%d" % (zoom_level, x, y) )
    tile_node.attachNewNode(node)

    # tile_node.setTwoSided(True)
    # tile_node.setAttrib(ShadeModelAttrib.make(ShadeModelAttrib.MFlat))

    # didn't seem to do anything? tile_node.setAttrib(LightRampAttrib.makeHdr2())

    if style == "satellite":
        tile_node.setMaterial(sat_mat)
        tile_node.setTexture(surface_tex)
    elif style == "polygon":
        tile_node.setMaterial(poly_mat)
    elif style == "wireframe":
        tile_node.setMaterial(poly_mat)
        tile_node.setRenderModeWireframe()
        tile_node.setTransparency(TransparencyAttrib.MAlpha)

    return { "name": "%d/%d/%d" % (zoom_level, x, y),
             "index": (zoom_level, x, y),
             "node": tile_node,
             "center_lla": center_lla,
             "children": {},
            }
