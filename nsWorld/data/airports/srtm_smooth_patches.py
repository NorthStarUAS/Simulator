# from math import atan2, cos, pi, radians, sin, sqrt
from matplotlib import pyplot as plt
import numpy as np
# import os
# import pathlib
from scipy import signal

# import navpy
from nsWorld.constants import ft2m
from nsWorld import srtm

# srtm_dir = os.path.join(pathlib.Path.home(), ".scenery_viewer", "cache", "srtm")
# pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)
# srtm_cache = srtm.Cache(srtm_dir)

by_tile = {}
def sortapt(apt):
    # expects a single airport
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
    # id = None
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

    # patch = srtm.SmoothPatch(srtm_cache, lat_min, lat_max, lon_min, lon_max, local_nedref)

    # add this airport info to srtm tile this region spans
    lat1, lon1, lat2, lon2 = srtm.gen_tile_range(lat_min, lon_max, lat_max, lon_min)
    print("lla range:", lat_min, lon_max, lat_max, lon_min)
    print("srtm tile range:", lat1, lon1, lat2, lon2)
    for lat in range(lat1, lat2+1):
        for lon in range(lon1, lon2+1):
            tilename = srtm.make_tile_name(lat, lon)
            print("srtm_tile:", tilename)
            if tilename not in by_tile:
                by_tile[tilename] = []
            by_tile[tilename].append([lat_min, lat_max, lon_min, lon_max])