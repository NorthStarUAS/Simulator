from math import atan2, cos, pi, radians, sin, sqrt
from matplotlib import pyplot as plt
import numpy as np
import os
import pathlib
from scipy import signal

import navpy
from nsWorld.constants import ft2m, r2d
from nsWorld import srtm

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
