#!/usr/bin/env python3

import gzip
import json
import os
import pathlib
import pickle

import sys
sys.path.append("../..")
import world.slippy_tiles as tiles

import srtm_apt
import genapt

# slippy map zoom level
zoom_level = 9

do_gen_apt_models = False
do_tiles_with_runways = False
do_srtm_runways = True

def save_airport(node, info):
    x, y = tiles.deg2num(info["nedref"][0], info["nedref"][1], zoom_level)
    base_dir = os.path.join(pathlib.Path.home(), ".scenery_viewer", "cache", "airports")
    dir = os.path.join(base_dir, "%d" % zoom_level, "%d" % x, "%d" % y)
    #dir = tiles.get_path(lat, lon, zoom_level)
    pathlib.Path(dir).mkdir(parents=True, exist_ok=True)
    path = os.path.join(dir, info["id"])
    print("saving:", info["id"], apt, info["nedref"], "->", path)
    with open(path + ".json", "w") as jsonfile:
        json.dump(info, jsonfile)
    with gzip.open(path, "wt") as f:
        for line in apt:
            f.write(line + "\n")
    node.writeBamFile(path + ".bam")

with gzip.open("apt.dat.gz", "r") as f:
    skipping = False
    start_at_id = "...."
    in_apt = False
    apt = []
    id = None
    has_runways = False
    for line in f:
        # print("line:", line)
        # tokens = line.decode().split()
        tokens = str(line, 'UTF-8', errors='ignore').split()
        #print("  tokens:", tokens)

        if len(tokens):
            if tokens[0] == "1":
                # airport definition
                id = tokens[4]
                alt_ft = float(tokens[1])
                name = " ".join(tokens[5:])
                print("Start of airport:", id, alt_ft, name)
                in_apt = True
                apt = []
                # lats = []
                # lons = []
            elif tokens[0] == "100":
                has_runways = True
                # # runway definition
                # lats.append(float(tokens[9]))
                # lats.append(float(tokens[18]))
                # lons.append(float(tokens[10]))
                # lons.append(float(tokens[19]))
            apt.append(" ".join(tokens))
        elif not len(tokens):
            if in_apt:
                # end of aiport definition
                if skipping and id == start_at_id:
                    skipping = False
                if has_runways and not skipping:
                    boundaries = False
                    if id == "WX46" or id == "KRAS" or id == "KTOA":
                        # hack to avoid segfault in polygon clipping library
                        # with specific airports (mysteries of floating point?)
                        boundaries = False
                    if id == "NZSP":
                        # just skip (overlaps /so/ many tiles!)
                        continue

                    if do_gen_apt_models:
                        node, info = genapt.genapt(apt, boundaries)
                        save_airport(node, info)
                    if do_tiles_with_runways:
                        srtm_apt.fitapt(apt)
                    if do_srtm_runways:
                        srtm_apt.sortapt(apt)
                if False and id == "KDLH":
                    tiles.get_paths_in_radius(lat, lon, zoom_level, 100000)
                    x, y = tiles.deg2num(lat, lon, zoom_level)
                    dx_deg, dy_deg, dx_m, dy_m = tiles.get_tile_size(x, y, zoom_level)
                    print("tile size:", dx_m, dy_m)
                    quit()
                if False and id == "LECO":
                    break

                in_apt = False

if do_tiles_with_runways:
    path = "tiles_with_runways.pkl"
    print("saving tiles with runways:", path)
    with open(path, "w") as f:
        pickle.dump(genapt.tiles_with_rwys, f)
if do_srtm_runways:
    path = "srtm_runways.pkl"
    print("saving srtm tiles vs runways:", path)
    print(srtm_apt.by_tile)
    print(type(srtm_apt.by_tile))
    with open(path, "wb") as f:
        pickle.dump(srtm_apt.by_tile, f)
    for key in srtm_apt.by_tile:
        print(len(srtm_apt.by_tile[key]), key, "srtm_count:")
