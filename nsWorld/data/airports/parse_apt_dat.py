#!/usr/bin/env python3

import argparse
import gzip
import json
import numpy as np
import os
import pathlib
import pickle

from nsWorld import slippy_tiles, srtm

import genapt
import overlapping_tiles
import srtm_smooth_patches

parser = argparse.ArgumentParser(description="Parse apt.dat file and do stuff.")
parser.add_argument("aptdat", help="path to apt.dat.gz (or apt.dat.ws3.gz) file")
parser.add_argument("--task", required=True, choices=["apt-models", "tiles-with-runways", "srtm-smooth-patches"], help="select the task to perform")
parser.add_argument("--start-id", help="begin processing at specified apt id")
parser.add_argument("--end-id", help="end processing at specified apt id")
parser.add_argument("--tile", help="process airports for this srtm tile, ex: N24W081")
args = parser.parse_args()

# slippy map zoom level
zoom_level = 9

do_gen_apt_models = False

def save_airport(node, info):
    x, y = slippy_tiles.deg2num(info["nedref"][0], info["nedref"][1], zoom_level)
    base_dir = os.path.join(pathlib.Path.home(), ".nsWorld", "cache", "airports")
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

with gzip.open(args.aptdat, "r") as f:
    if args.start_id is not None:
        skipping = True
    else:
        skipping = False
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
                lats = []
                lons = []
            elif tokens[0] == "100":
                has_runways = True
                # # runway definition
                lats.append(float(tokens[9]))
                lats.append(float(tokens[18]))
                lons.append(float(tokens[10]))
                lons.append(float(tokens[19]))
            apt.append(" ".join(tokens))
        elif not len(tokens):
            if in_apt:
                # end of aiport definition
                if skipping and id == args.start_id:
                    skipping = False
                    print("Begin processing at:", args.start_id)
                if has_runways and not skipping:
                    boundaries = False
                    if args.tile:
                        if not len(lats):
                            continue
                        lat = np.mean(lats)
                        lon = np.mean(lons)
                        if args.tile != srtm.make_tile_name(lat, lon):
                            continue
                    if id == "WX46" or id == "KRAS" or id == "KTOA":
                        # hack to avoid segfault in polygon clipping library
                        # with specific airports (mysteries of floating point?)
                        boundaries = False
                    if id == "NZSP":
                        # just skip (overlaps /so/ many tiles!)
                        continue

                    if args.task == "apt-models":
                        node, info = genapt.genapt(apt)
                        save_airport(node, info)
                    if args.task == "tiles-with-runways":
                        overlapping_tiles.flag_airport(apt)
                    if args.task == "srtm-smooth-patches":
                        srtm_smooth_patches.sortapt(apt)
                if id == args.end_id:
                    skipping = True
                    print("End processing at:", args.start_id)
                if False and id == "KDLH":
                    tiles.get_paths_in_radius(lat, lon, zoom_level, 100000)
                    x, y = tiles.deg2num(lat, lon, zoom_level)
                    dx_deg, dy_deg, dx_m, dy_m = tiles.get_tile_size(x, y, zoom_level)
                    print("tile size:", dx_m, dy_m)
                    quit()
                in_apt = False

if args.task == "tiles-with-runways":
    path = "tiles_with_runways.pkl"
    print("saving tiles with runways:", path)
    with open(path, "wb") as f:
        pickle.dump(overlapping_tiles.tiles_with_rwys, f)
if args.task == "srtm-smooth-patches":
    path = "smooth_patches.pkl"
    print("saving srtm smooth patches:", path)
    with open(path, "wb") as f:
        pickle.dump(srtm_smooth_patches.by_tile, f)
