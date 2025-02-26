from math import atan2, cos, pi, radians, sin
from matplotlib import pyplot as plt
import numpy as np
from Polygon import *

import navpy
from nstSimulator.utils.constants import ft2m, r2d
from nstSimulator import slippy_tiles

# ginormous tree of tiles that have runway overlap
tiles_with_rwys = {}
for level in range(9,19+1):
    tiles_with_rwys[level] = {}

def flag_overlapping_tiles(runway):
    do_plot = False

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
        if do_plot:
            fig, ax1 = plt.subplots(1, 1)
            ax1.grid(True)
            ax1.axis("equal")

        ll0 = runway[0][:2]
        ll1 = runway[1][:2]
        ll2 = runway[2][:2]
        ll3 = runway[3][:2]
        if do_plot:
            ax1.plot([ll0[1], ll1[1], ll3[1], ll2[1], ll0[1]], [ll0[0], ll1[0], ll3[0], ll2[0], ll0[0]], "-")

        minx, maxy = slippy_tiles.deg2num(min_lat, min_lon, level)
        maxx, miny = slippy_tiles.deg2num(max_lat, max_lon, level)
        print("  level:", level, minx, miny, maxx, maxy)
        if do_plot:
            ax1.plot([min_lon, max_lon, max_lon, min_lon, min_lon], [max_lat, max_lat, min_lat, min_lat, max_lat], "-")
        top = tiles_with_rwys[level]
        for x in range(minx,maxx+1):
            for y in range(miny,maxy+1):
                nw_lat, nw_lon = slippy_tiles.num2deg(x, y, level)
                se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, level)
                tile_poly = Polygon([[nw_lat, nw_lon], [nw_lat, se_lon], [se_lat, se_lon], [se_lat, nw_lon]])
                if len(tile_poly & runway_poly):
                    if do_plot:
                        ax1.plot([nw_lon, se_lon, se_lon, nw_lon, nw_lon], [nw_lat, nw_lat, se_lat, se_lat, nw_lat], "-")
                    print("    overlap!", x, y)
                    if x not in top:
                        top[x] = {}
                    top[x][y] = 1
        if do_plot:
            plt.show()

def flag_airport(apt):
    # expects a single airport per file
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

    # 10% expansion (5% on each margin)
    lat_extend = (lat_max - lat_min) * 0.05
    lon_extend = (lon_max - lon_min) * 0.05
    lat_min -= lat_extend
    lat_max += lat_extend
    lon_min -= lon_extend
    lon_max += lon_extend

    print("coverage:", lat_min, lat_max, lon_min, lon_max)

    # rotate a e, n vector (x, y) by the angle
    def ned_rotate(ned, angle_rad, n, e):
        x = e * cos(angle_rad) - n * sin(angle_rad)
        y = e * sin(angle_rad) + n * cos(angle_rad)
        return [ned[0] + y, ned[1] + x, ned[2]]

    for runway in runways:
        # grab values from runway record
        width = float(runway[1])
        lla1 = [float(runway[9]), float(runway[10]), alt_m]
        over1 = float(runway[12])
        lla2 = [float(runway[18]), float(runway[19]), alt_m]
        over2 = float(runway[21])
        nedref = 0.5 * (np.array(lla1) + np.array(lla2))
        print("  nedref:", nedref, "over1:", over1, "over2:", over2)

        # ned location of two end points (includes displaced threshold, but not overrun/blastpad)
        ned1 = navpy.lla2ned(lla1[0], lla1[1], lla1[2], nedref[0], nedref[1], nedref[2])
        ned2 = navpy.lla2ned(lla2[0], lla2[1], lla2[2], nedref[0], nedref[1], nedref[2])
        angle2 = atan2(ned2[0]-ned1[0], ned2[1]-ned1[1])
        length = np.linalg.norm(ned1-ned2)
        print("  angle:", angle2, "len:", length)

        # ned corners including overrun/blastpad and extra margin
        l_margin = length * 0.05  # 5% added to each end (10% total)
        w_margin = width * 0.25   # 25% added to each edge (50% total)
        c1 = ned_rotate(ned1, angle2, (0.5*width + w_margin), -(over1 + l_margin))
        c2 = ned_rotate(ned1, angle2, -(0.5*width + w_margin), -(over1 + l_margin))
        c3 = ned_rotate(ned2, angle2, (0.5*width + w_margin), (over2 + l_margin))
        c4 = ned_rotate(ned2, angle2, -(0.5*width + w_margin), (over2 + l_margin))
        print("  corner1:", list(ned1), c1)
        print("  corner2:", list(ned1), c2)
        print("  corner3:", list(ned2), c3)
        print("  corner4:", list(ned2), c4)

        if False:
            w_scale = 1.5
            l_scale = 1.2
            w2 = w_scale * float(runway[1]) * 0.5
            # angle = 0.5*pi - atan2(ned2[0]-ned1[0], ned2[1]-ned1[1])
            print("angle:", angle, "angle2:", angle2)
            length = l_scale * np.linalg.norm(ned1-ned2)
            print(ned1, ned2, angle*r2d, length)

            # generate runway corners (in ned space)
            c1 = [ ned1[1] + sin(angle-0.5*pi)*w2, ned1[0] + cos(angle-0.5*pi)*w2 ]
            c2 = [ ned1[1] - sin(angle-0.5*pi)*w2, ned1[0] - cos(angle-0.5*pi)*w2 ]
            c3 = [ ned2[1] + sin(angle-0.5*pi)*w2, ned2[0] + cos(angle-0.5*pi)*w2 ]
            c4 = [ ned2[1] - sin(angle-0.5*pi)*w2, ned2[0] - cos(angle-0.5*pi)*w2 ]

        # convert actual runway corners back to lla
        c1_lla = navpy.ned2lla([c1[0], c1[1], c1[2]], nedref[0], nedref[1], nedref[2])
        c2_lla = navpy.ned2lla([c2[0], c2[1], c2[2]], nedref[0], nedref[1], nedref[2])
        c3_lla = navpy.ned2lla([c3[0], c3[1], c3[2]], nedref[0], nedref[1], nedref[2])
        c4_lla = navpy.ned2lla([c4[0], c4[1], c4[2]], nedref[0], nedref[1], nedref[2])

        flag_overlapping_tiles([c1_lla, c2_lla, c3_lla, c4_lla])
