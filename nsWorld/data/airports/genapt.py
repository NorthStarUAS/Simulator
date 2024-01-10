from math import atan2, cos, pi, radians, sin
from matplotlib import pyplot as plt
import numpy as np
import os
import pathlib
from Polygon import *

from panda3d.core import *

import navpy
from nsWorld.constants import ft2m, r2d
from nsWorld import slippy_tiles

from polygon3d import Polygon3d

# test/debug
# import matplotlib.pyplot as plt

# share material definitions
#
#  note to self: could I disable lighting and set the color directly?
rwy_mat = Material()
rwy_mat.setAmbient((0.8, 0.8, 0.8, 0.75))
rwy_mat.setDiffuse((0.8, 0.8, 0.8, 0.75))

taxi_mat = Material()
taxi_mat.setAmbient((0.5, 0.5, 0.5, 0.75))
taxi_mat.setDiffuse((0.5, 0.5, 0.5, 0.75))

bound_mat = Material()
bound_mat.setAmbient((0.5, 0.7, 0.5, 0.5))
bound_mat.setDiffuse((0.5, 0.7, 0.5, 0.5))

rwy_format = GeomVertexFormat.getV3()

# ginormous tree of tiles that have runway overlap
tiles_with_rwys = {}
for level in range(9,19+1):
    tiles_with_rwys[level] = {}

def flag_overlapping_tiles(runway):
    # make polygons in lla space just to test intersection
    ll0 = runway[0][:2]
    ll1 = runway[1][:2]
    ll2 = runway[2][:2]
    ll3 = runway[3][:2]
    runway_poly = Polygon([ll0, ll1, ll3, ll2])

    min_lat, min_lon, max_lat, max_lon = 360, 360, -360, -360
    for c in runway:
        if c[0] < min_lat: min_lat = c[0]
        if c[0] > max_lat: max_lat = c[0]
        if c[1] < min_lon: min_lon = c[1]
        if c[1] > max_lon: max_lon = c[1]
    print("min/max:", min_lat, min_lon, max_lat, max_lon)

    for level in range(9,19+1):
        # test/debug
        # fig, ax1 = plt.subplots(1, 1)
        # ax1.grid(True)
        # ax1.axis("equal")

        ll0 = runway[0][:2]
        ll1 = runway[1][:2]
        ll2 = runway[2][:2]
        ll3 = runway[3][:2]
        # ax1.plot([ll0[1], ll1[1], ll3[1], ll2[1], ll0[1]], [ll0[0], ll1[0], ll3[0], ll2[0], ll0[0]], "-")

        minx, maxy = slippy_tiles.deg2num(min_lat, min_lon, level)
        maxx, miny = slippy_tiles.deg2num(max_lat, max_lon, level)
        print("  level:", level, minx, miny, maxx, maxy)
        # ax1.plot([min_lon, max_lon, max_lon, min_lon, min_lon], [max_lat, max_lat, min_lat, min_lat, max_lat], "-")
        top = tiles_with_rwys[level]
        for x in range(minx,maxx+1):
            for y in range(miny,maxy+1):
                nw_lat, nw_lon = slippy_tiles.num2deg(x, y, level)
                se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, level)
                tile_poly = Polygon([[nw_lat, nw_lon], [nw_lat, se_lon], [se_lat, se_lon], [se_lat, nw_lon]])
                if len(tile_poly & runway_poly):
                    # ax1.plot([nw_lon, se_lon, se_lon, nw_lon, nw_lon], [nw_lat, nw_lat, se_lat, se_lat, nw_lat], "-")
                    print("    overlap!", x, y)
                    if x not in top:
                        top[x] = {}
                    top[x][y] = 1
        # plt.show()

from scipy import signal
step_size = 25
cutoff_freq = step_size * 0.002  # bigger values == tighter fit
b, a = signal.butter(2, cutoff_freq, 'lowpass')

from nsWorld import srtm
dot_root = ".nsWorld"
srtm_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "srtm")
pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)
srtm_cache = srtm.Cache(srtm_dir)

def make_smooth_surface(lat_min, lat_max, lon_min, lon_max, nedref):
    nedmin = navpy.lla2ned(lat_min, lon_min, nedref[2], nedref[0], nedref[1], nedref[2])
    nedmax = navpy.lla2ned(lat_max, lon_max, nedref[2], nedref[0], nedref[1], nedref[2])
    print("nedmin:", nedmin)
    print("nedmax:", nedmax)
    ndivs = int((nedmax[0] - nedmin[0]) / step_size) + 1
    edivs = int((nedmax[1] - nedmin[1]) / step_size) + 1
    print("divs n/e:", ndivs, edivs)
    heights = np.array( (ndivs+1, edivs+1) )

    # sample the airport coverate area at high density
    fit_pts = []
    for lat in np.linspace(lat_min, lat_max, ndivs+1):
        for lon in np.linspace(lon_min, lon_max, edivs+1):
            fit_pts.append( (lon, lat) )
    vals = [ nedref[2] ] * len(fit_pts)
    lat1, lon1, lat2, lon2 = srtm.gen_tile_range(lat_min, lon_max, lat_max, lon_min)
    for lat in range(lat1, lat2+1):
        for lon in range(lon1, lon2+1):
            srtm_tile = srtm_cache.get_tile(lat, lon)
            tilename = srtm.make_tile_name(lat, lon)
            # srtm_cache.level_runways(tilename) # if needed
            if srtm_tile is not None:
                zs = srtm_tile.raw_interpolate(np.array(fit_pts))
                #print zs
                # copy the good altitudes back to the corresponding ned points
                if len(zs) == len(fit_pts):
                    for i in range(len(fit_pts)):
                        if zs[i] > -10000:
                            # llas[i][2] = zs[i]
                            vals[i] = zs[i]
    # print("raw vals:", vals)
    raw_vals = np.array(vals).reshape( (ndivs+1, edivs+1))
    print("raw_vals reshape:", raw_vals)

    # smooth data by row
    by_row = raw_vals.copy()
    for r in range(ndivs+1):
        by_row[r,:] = signal.filtfilt(b, a, raw_vals[r,:])
    # smooth data by col
    by_col = raw_vals.copy()
    for c in range(edivs+1):
        by_col[:,c] = signal.filtfilt(b, a, raw_vals[:,c])
        # plt.figure()
        # plt.plot(raw_vals[:,c], ".")
        # plt.plot(by_col[:,c])
        # # plt.axis("equal")
        # plt.xlabel("pos")
        # plt.ylabel("elev (m)")
        # plt.title(id)
        # plt.show()
    # average the two (making this up as I go along!)
    smooth = (by_row + by_col) * 0.5
    plt.figure()
    plt.imshow(raw_vals, origin="lower")
    plt.title("raw")
    plt.figure()
    plt.imshow(by_row, origin="lower")
    plt.title("by_row")
    plt.figure()
    plt.imshow(by_col, origin="lower")
    plt.title("by_col")
    plt.figure()
    plt.imshow(smooth, origin="lower")
    plt.title("smooth")
    plt.show()


def interpolate_terrain(vts, nedref):
    # inefficient, but we have to convert ned vts's back to lla before doing our interpolation work
    llas = []
    lat_min = 360
    lat_max = -360
    lon_min = 360
    lon_max = -360
    for ned in vts:
        lla = navpy.ned2lla(ned, nedref[0], nedref[1], nedref[2])
        if lla[0] < lat_min: lat_min = lla[0]
        if lla[0] > lat_max: lat_max = lla[0]
        if lla[1] < lon_min: lon_min = lla[1]
        if lla[1] > lon_max: lon_max = lla[1]
        llas.append([lla[1], lla[0]]) # this has to be lon,lat (not lat,lon) here
    lat1, lon1, lat2, lon2 = srtm.gen_tile_range(lat_min, lon_max, lat_max, lon_min)
    print("vts range:", lat1, lon1, lat2, lon2)
    for lat in range(lat1, lat2+1):
        for lon in range(lon1, lon2+1):
            print("vts working on:", lat, lon)
            srtm_tile = srtm_cache.get_tile(lat, lon)
            tilename = srtm.make_tile_name(lat, lon)
            srtm_cache.level_runways(tilename) # if needed
            if srtm_tile is not None:
                zs = srtm_tile.interpolate(np.array(llas))
                #print zs
                # copy the good altitudes back to the corresponding ned points
                if len(zs) == len(llas):
                    for i in range(len(llas)):
                        if zs[i] > -10000:
                            # llas[i][2] = zs[i]
                            vts[i][2] = zs[i]
    print("vts inside:", vts)

def genapt(apt, do_boundaries):
    # expects a single airport per file
    info = {}
    runways = []
    taxiways_lla = []
    taxiways_old = []
    taxi = []
    boundaries_lla = []
    boundary = []
    id = None
    in_taxi = False
    in_boundary = False
    lats = []
    lons = []
    for line in apt:
        #print(line)
        tokens = line.split()
        if len(tokens):
            if tokens[0] == "1":
                # airport definition
                id = tokens[4]
                alt_ft = float(tokens[1])
                alt_m = alt_ft * ft2m
                name = " ".join(tokens[5:])
                print("Start of airport:", id, alt_ft, name)
                info["id"]= id
                info["name"] = name
                info["alt_ft"] = alt_ft
                apt = []
            elif tokens[0] == "100":
                # runway definition
                print("  runway:", tokens)
                runways.append(tokens)
            elif tokens[0] == "110":
                # start of new taxiway definition
                taxi = []
                in_taxi = True
            elif tokens[0] == "111":
                # node point
                if in_taxi:
                    taxi.append( [float(tokens[1]), float(tokens[2])] )
                elif in_boundary:
                    boundary.append( [float(tokens[1]), float(tokens[2])] )
            elif tokens[0] == "112":
                # bezier curve (currently just handling them as points)
                if in_taxi:
                    taxi.append( [float(tokens[1]), float(tokens[2])] )
                    taxi.append( [float(tokens[3]), float(tokens[4])] )
                elif in_boundary:
                    boundary.append( [float(tokens[1]), float(tokens[2])] )
                    boundary.append( [float(tokens[3]), float(tokens[4])] )
            elif tokens[0] == "113":
                # last point in a contour
                if in_taxi:
                    taxi.append( [float(tokens[1]), float(tokens[2])] )
                    taxiways_lla.append(taxi)
                elif in_boundary:
                    boundary.append( [float(tokens[1]), float(tokens[2])] )
                    boundaries_lla.append(boundary)
                in_taxi = False
                in_boundary = False
            elif tokens[0] == "114":
                # last point in a contour
                if in_taxi:
                    taxi.append( [float(tokens[1]), float(tokens[2])] )
                    taxi.append( [float(tokens[3]), float(tokens[4])] )
                    taxiways_lla.append(taxi)
                elif in_boundary:
                    boundary.append( [float(tokens[1]), float(tokens[2])] )
                    boundary.append( [float(tokens[3]), float(tokens[4])] )
                    boundaries_lla.append(boundary)
                in_taxi = False
                in_boundary = False
            elif tokens[0] == "130":
                # start of airport boundary definition
                boundary = []
                in_boundary = True
            elif tokens[0] == "10":
                # old x-plane taxiway defintion
                taxiways_old.append(tokens)
    # end of aiport definition

    if not len(runways):
        return None

    # estimate max/min lla coverage and a central-ish reference point for the
    # airport.
    lats = []
    lons = []
    for runway in runways:
        lats.append(float(runway[9]))
        lats.append(float(runway[18]))
        lons.append(float(runway[10]))
        lons.append(float(runway[19]))
    for taxiway in taxiways_lla:
        for p in taxiway:
            lats.append(p[0])
            lons.append(p[1])
    for taxiway in taxiways_old:
        lats.append(float(taxiway[1]))
        lons.append(float(taxiway[2]))
    lat_min = np.min(lats)
    lat_max = np.max(lats)
    lon_min = np.min(lons)
    lon_max = np.max(lons)
    local_nedref = [ (lat_min + lat_max)*0.5, (lon_min + lon_max) * 0.5, alt_m ]
    print("coverage:", lat_min, lat_max, lon_min, lon_max)
    print("nedref:", local_nedref)
    info["nedref"] = local_nedref
    make_smooth_surface(lat_min, lat_max, lon_min, lon_max, local_nedref)

    #airport_node = render.attachNewNode(id)
    airport_node = NodePath(id)
    airport_node.setTransparency(TransparencyAttrib.MAlpha)

    runway_node = airport_node.attachNewNode("runways")
    runway_node.setTwoSided(True)
    runway_node.setMaterial(rwy_mat)
    runway_node.setTransparency(TransparencyAttrib.MAlpha)

    taxiway_node = airport_node.attachNewNode("taxiways")
    taxiway_node.setTwoSided(True)
    taxiway_node.setMaterial(taxi_mat)
    taxiway_node.setTransparency(TransparencyAttrib.MAlpha)

    boundary_node = airport_node.attachNewNode("boundaries")
    boundary_node.setTwoSided(True)
    boundary_node.setMaterial(bound_mat)
    boundary_node.setTransparency(TransparencyAttrib.MAlpha)

    # whole airport needs to be positioned in the wider ned system
    #apt_pos = navpy.lla2ned(local_nedref[0], local_nedref[1], local_nedref[2], comms_mgr.nedref[0], comms_mgr.nedref[1], comms_mgr.nedref[2])
    #airport_node.setPos(apt_pos[1], apt_pos[0], -apt_pos[2])

    clip_poly = Polygon()
    info["runways_lla"] = []
    for runway in runways:
        w2 = float(runway[1]) * 0.5
        ned1 = navpy.lla2ned(float(runway[9]), float(runway[10]), alt_m, local_nedref[0], local_nedref[1], local_nedref[2])
        ned2 = navpy.lla2ned(float(runway[18]), float(runway[19]), alt_m, local_nedref[0], local_nedref[1], local_nedref[2])
        angle = 0.5*pi - atan2(ned2[0]-ned1[0], ned2[1]-ned1[1])
        length = np.linalg.norm(ned1-ned2)
        divs = int(round(length / 200)) + 1
        if divs < 1:
            divs = 1
        print(ned1, ned2, angle*r2d, length)

        # generate runway in segments (for surface/terrain fitting and to work
        # around elstupido triangulator algorithms that generates super long
        # skinny triangles)
        edge1 = []
        edge2 = []
        for p in zip(np.linspace(ned1[1], ned2[1], divs+1), np.linspace(ned1[0], ned2[0], divs+1)):
            e1 = [ p[0] + sin(angle-0.5*pi)*w2, p[1] + cos(angle-0.5*pi)*w2 ]
            e2 = [ p[0] - sin(angle-0.5*pi)*w2, p[1] - cos(angle-0.5*pi)*w2 ]
            edge1.append(e1)
            edge2.append(e2)
        for i in range(len(edge1)-1):
            c1 = edge1[i]
            c2 = edge2[i]
            c3 = edge1[i+1]
            c4 = edge2[i+1]
            print(c1, c2, c4, c3)
            raw_segment = Polygon([c1, c2, c4, c3])
            segment = raw_segment - clip_poly
            clip_poly = clip_poly + raw_segment
            for i, contour in enumerate(segment):
                # contour = rwy_poly.contour(i)
                print("  contour:", contour, segment.isHole(i))
                vts = []
                for p in contour:
                    vts.append( [p[0], p[1], alt_m])
                print("vts before:", vts)
                interpolate_terrain(vts, local_nedref)
                print("vts after srtm:", vts)
                p3 = Polygon3d(vts)
                runway_node.attachNewNode(p3.makeNode())
        # convert actual runway corners back to lla
        c1_lla = navpy.ned2lla([c1[1], c1[0], -alt_m], local_nedref[0], local_nedref[1], 0)
        c2_lla = navpy.ned2lla([c2[1], c2[0], -alt_m], local_nedref[0], local_nedref[1], 0)
        c3_lla = navpy.ned2lla([c3[1], c3[0], -alt_m], local_nedref[0], local_nedref[1], 0)
        c4_lla = navpy.ned2lla([c4[1], c4[0], -alt_m], local_nedref[0], local_nedref[1], 0)
        info["runways_lla"].append([c1_lla, c2_lla, c3_lla, c4_lla])
        flag_overlapping_tiles([c1_lla, c2_lla, c3_lla, c4_lla])
    print("clip poly (after runways):", clip_poly)

    # print("taxi sections:", len(taxiways_lla))
    # print("taxi lla:", taxiways_lla)
    count = len(taxiways_lla)
    for i, taxi_lla in enumerate(taxiways_lla):
        if count > 250:
            print("taxi:", i, "of", count)
        # convert to ned
        taxiway = []
        for pt in taxi_lla:
            ned = navpy.lla2ned(pt[0], pt[1], alt_m, local_nedref[0], local_nedref[1], local_nedref[2])
            taxiway.append( [ned[1], ned[0]] )
        # print("taxi (ned):", taxiway)
        raw_segment = Polygon(taxiway)
        segment = raw_segment - clip_poly
        clip_poly = clip_poly + raw_segment
        for i, contour in enumerate(segment):
            print("  taxi contour:", contour, segment.isHole(i))
            vts = []
            for p in contour:
                vts.append( [p[0], p[1], alt_m])
            interpolate_terrain(vts, local_nedref)
            p3 = Polygon3d(vts)
            taxiway_node.attachNewNode(p3.makeNode())
    print("clip poly (after new taxiway polygons):", clip_poly)

    for taxiway in taxiways_old:
        lat = float(taxiway[1])
        lon = float(taxiway[2])
        hdg = float(taxiway[4])
        length = float(taxiway[5]) * ft2m
        width = float(taxiway[8]) * ft2m
        nedc = navpy.lla2ned(lat, lon, alt_m, local_nedref[0], local_nedref[1], local_nedref[2])
        w2 = width * 0.5
        l2 = length * 0.5
        angle = radians(90 - hdg)
        divs = int(round(length / 200)) + 1
        if divs < 1:
            divs = 1
        print("taxi:", nedc, l2, w2, angle*r2d, divs)
        def rotate(x, y, angle, cx, cy):
            x_r = cos(angle) * x - sin(angle) * y
            y_r = sin(angle) * x + cos(angle) * y
            return [x_r + cx, y_r + cy]
        c1 = rotate(-l2, -w2, angle, nedc[1], nedc[0])
        c2 = rotate( l2, -w2, angle, nedc[1], nedc[0])
        c3 = rotate(-l2,  w2, angle, nedc[1], nedc[0])
        c4 = rotate( l2,  w2, angle, nedc[1], nedc[0])
        print(c1, c2, c3, c4)

        # generate taxiway in segments (for surface/terrain fitting and to work
        # around elstupido triangulator algorithms that generates super long
        # skinny triangles)
        edge1 = []
        edge2 = []
        for e1 in zip(np.linspace(c1[0], c2[0], divs+1), np.linspace(c1[1], c2[1], divs+1)):
            edge1.append(e1)
        for e2 in zip(np.linspace(c3[0], c4[0], divs+1), np.linspace(c3[1], c4[1], divs+1)):
            edge2.append(e2)
        for i in range(len(edge1)-1):
            c1 = edge1[i]
            c2 = edge2[i]
            c3 = edge1[i+1]
            c4 = edge2[i+1]
            print(c1, c2, c4, c3)
            raw_segment = Polygon([c1, c2, c4, c3])
            segment = raw_segment - clip_poly
            clip_poly = clip_poly + raw_segment
            for i, contour in enumerate(segment):
                # contour = rwy_poly.contour(i)
                print("  contour:", contour, segment.isHole(i))
                vts = []
                for p in contour:
                    vts.append( [p[0], p[1], alt_m])
                interpolate_terrain(vts, local_nedref)
                p3 = Polygon3d(vts)
                taxiway_node.attachNewNode(p3.makeNode())
    print("clip poly (after old taxiways):", clip_poly)

    if False:
        # don't overlap runways
        taxiways_poly = taxiways_poly - runways_poly

        print("taxiways poly:", taxiways_poly)
        for tri in taxiways_poly.triStrip():
            print("tristrip:", tri)
            vdata = GeomVertexData("taxiway", rwy_format, Geom.UHStatic)
            vdata.setNumRows(len(tri))

            vertex = GeomVertexWriter(vdata, "vertex")
            for t in tri:
                vertex.addData3(t[0], t[1], 0)

            prim = GeomTristrips(Geom.UHStatic) # don't expect geometry to change
            prim.add_consecutive_vertices(0, len(tri))
            prim.closePrimitive()

            geom = Geom(vdata)
            geom.addPrimitive(prim)

            node = GeomNode("geom")
            node.addGeom(geom)
            taxiway_node.attachNewNode(node)

        if do_boundaries:
            print("boundaries:")
            boundaries_poly = Polygon()
            for boundary_lla in boundaries_lla:
                # convert to ned
                boundary = []
                for pt in boundary_lla:
                    ned = navpy.lla2ned(pt[0], pt[1], alt_m, local_nedref[0], local_nedref[1], local_nedref[2])
                    boundary.append( [ned[1], ned[0]] )
                boundary_poly = Polygon(boundary)
                boundaries_poly = boundaries_poly + boundary_poly
            print("boundaries 1:", boundaries_poly)

            # don't overlap runways or taxiways
            boundaries_poly = boundaries_poly - runways_poly
            print("boundaries 2:", boundaries_poly)
            boundaries_poly = boundaries_poly - taxiways_poly
            print("boundaries 3:", boundaries_poly)

            print("boundaries poly:", boundaries_poly)
            for tri in boundaries_poly.triStrip():
                print("tristrip:", tri)
                vdata = GeomVertexData("boundary", rwy_format, Geom.UHStatic)
                vdata.setNumRows(len(tri))

                vertex = GeomVertexWriter(vdata, "vertex")
                for t in tri:
                    vertex.addData3(t[0], t[1], 0)

                prim = GeomTristrips(Geom.UHStatic) # don't expect geometry to change
                prim.add_consecutive_vertices(0, len(tri))
                prim.closePrimitive()

                geom = Geom(vdata)
                geom.addPrimitive(prim)

                node = GeomNode("geom")
                node.addGeom(geom)
                boundary_node.attachNewNode(node)

    return airport_node, info