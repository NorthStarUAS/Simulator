from math import atan2, cos, pi, radians, sin, sqrt
from matplotlib import pyplot as plt
import numpy as np
import os
import pathlib
from scipy import signal
import sys

import navpy
sys.path.append("../..")
from lib.constants import ft2m, r2d
from world import srtm

srtm_dir = os.path.join(pathlib.Path.home(), ".scenery_viewer", "cache", "srtm")
pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)
srtm_cache = srtm.Cache(srtm_dir)

by_tile = {}
def sortapt(apt):
    # expects a single airport
    info = {}
    info["runways"] = []
    # id = None
    for line in apt:
        #print(line)
        tokens = str(line).split()
        if len(tokens):
            if tokens[0] == "1":
                # airport definition
                # id = tokens[4]
                alt_ft = float(tokens[1])
                alt_m = alt_ft * ft2m
                name = " ".join(tokens[5:])
                print("Start of airport:", id, alt_ft, name)
                info["id"]= tokens[4]
                info["name"] = name
                info["alt_ft"] = alt_ft
                apt = []
            elif tokens[0] == "100":
                # runway definition
                print("  runway:", tokens)
                info["runways"].append(tokens)

    if not len(info["runways"]):
        return None

    for runway in info["runways"]:
        # find the srtm tiles this runway spans
        lla1 = [float(runway[9]), float(runway[10]), alt_m]
        lla2 = [float(runway[18]), float(runway[19]), alt_m]
        lat_min = np.min([lla1[0], lla2[0]])
        lat_max = np.max([lla1[0], lla2[0]])
        lon_min = np.min([lla1[1], lla2[1]])
        lon_max = np.max([lla1[1], lla2[1]])
        lat1, lon1, lat2, lon2 = srtm.gen_tile_range(lat_min, lon_max, lat_max, lon_min)
        print("lla range:", lat_min, lon_max, lat_max, lon_min)
        print("srtm tile range:", lat1, lon1, lat2, lon2)

        # add this airport info to srtm tile this region spans
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                tilename = srtm.make_tile_name(lat, lon)
                print("srtm_tile:", tilename)
                if tilename not in by_tile:
                    by_tile[tilename] = []
                by_tile[tilename].append(runway)

# Goal:
# * input srtm tile and a list of runways that intersect that tile
# * sample a set of elevations along the length of each runway
# * filter/fit a smoother line along the length of each runway
# * use that filtered runway elevation to adjust nearby srtm elevation points to
#   smoothly blend the runway shape into the surrounding terrain.

segment_length = 25
cutoff_freq = segment_length * 0.005  # bigger values == tighter fit
b, a = signal.butter(2, cutoff_freq, 'lowpass')

class AdjustSRTM():
    def __init__(self):
        pass

def fitapt(apt):
    # expects a single airport per file
    info = {}
    runways = []
    id = None
    in_taxi = False
    in_boundary = False
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

    if not len(runways):
        return None

    # create a central-ish reference point for the airport by averaging the
    # runway center points.
    lats = []
    lons = []
    for runway in runways:
        lats.append(float(runway[9]))
        lats.append(float(runway[18]))
        lons.append(float(runway[10]))
        lons.append(float(runway[19]))
    local_nedref = [ np.mean(lats), np.mean(lons), alt_m ]
    print(local_nedref)
    info["nedref"] = local_nedref

    info["runways_lla"] = []
    for runway in runways:
        # leverage lla -> ned coordinate transformation to get real world length in meters
        w = float(runway[1])
        w2 = w * 0.5
        lla1 = [float(runway[9]), float(runway[10]), alt_m]
        lla2 = [float(runway[18]), float(runway[19]), alt_m]
        ned1 = navpy.lla2ned(lla1[0], lla1[1], lla1[2], local_nedref[0], local_nedref[1], local_nedref[2])
        ned2 = navpy.lla2ned(lla2[0], lla2[1], lla2[2], local_nedref[0], local_nedref[1], local_nedref[2])
        angle = 0.5*pi - atan2(ned2[0]-ned1[0], ned2[1]-ned1[1])
        print("neds:", ned1, ned2)
        l = sqrt( (ned2[1] - ned1[1])**2 + (ned2[0] - ned1[0])**2 )
        print("rwy:", ned1, ned2, angle*r2d, l)

        # 1. divide up the runway length in some reasonable # of segments 10m (5m, 1m?)

        divs = int(l / segment_length) + 1
        print("divs:", divs)
        lat_step = (lla2[0] - lla1[0]) / divs
        lon_step = (lla2[1] - lla1[1]) / divs
        print(lat_step, lon_step)
        fit_pts = []
        fit_vals = []
        xm = []
        print(lla1[0], lla2[0] + lat_step, lat_step)
        print(lla1[1], lla2[1] + lon_step, lon_step)
        lat = lla1[0]
        lon = lla1[1]
        for i in range(divs+1):
            # print("ll:", lat, lon)
            fit_pts.append([lon, lat])
            fit_vals.append(None)
            xm.append(i*segment_length)
            lat += lat_step
            lon += lon_step

        # 2. raw interpolate the elevation along the centerline at each of these subdivided points

        fit_pts = np.array(fit_pts)
        # print(fit_pts)
        lat_min = np.min(fit_pts[:,1])
        lat_max = np.max(fit_pts[:,1])
        lon_min = np.min(fit_pts[:,0])
        lon_max = np.max(fit_pts[:,0])
        lat1, lon1, lat2, lon2 = srtm.gen_tile_range(lat_min, lon_max, lat_max, lon_min)
        print("lla range:", lat_min, lon_max, lat_max, lon_min)
        print("srtm tile range:", lat1, lon1, lat2, lon2)

        # for each srtm tile this region spans, interpolate as many elevation values
        # as we can, then copy the good values into zs.  When we finish all the
        # loaded tiles, we should have found elevations for the entire range of
        # points.
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                srtm_tile = srtm_cache.get_tile(lat, lon)
                print("srtm_tile:", srtm_tile)
                if srtm_tile is not None:
                    zs = srtm_tile.lla_interpolate(np.array(fit_pts))
                    #print zs
                    # copy the good altitudes back to the corresponding ned points
                    if len(zs) == len(fit_pts):
                        for i in range(len(fit_pts)):
                            if zs[i] > -10000:
                                fit_vals[i] = zs[i]

        for i in range(len(fit_vals)):
            if fit_vals[i] is None:
                print("Problem interpolating elevation for:", fit_pts[i], "(ocean?)")
                fit_vals[i] = 0.0

        # 3. scipy.filtfilt() with suitable bandwidth values to get a reasonable smoothed surface.
        if len(fit_vals) > 10:
            # print("fit_vals:", len(fit_vals), fit_vals)
            elev_fit = signal.filtfilt(b, a, fit_vals)

            # plt.figure()
            # plt.plot(xm, fit_vals, ".")
            # plt.plot(xm, elev_fit)
            # # plt.axis("equal")
            # plt.xlabel("length (m)")
            # plt.ylabel("elev (m)")
            # plt.title(id)
            # plt.show()

        # 4. add to fits structure
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                srtm_tile = srtm_cache.get_tile(lat, lon)
                print("adding to srtm_tile:", srtm_tile)
                if srtm_tile not in fits:
                    fits[srtm_tile] = []
                fits[srtm_tile].append( [fit_pts, fit_vals])

        # 5. traverse nearby srtm posts and adjust elevation based on nearness to the fit line ...

    return None, info