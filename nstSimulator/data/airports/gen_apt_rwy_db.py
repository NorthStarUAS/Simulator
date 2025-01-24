#!/usr/bin/python3

# this module facilitates computing startup positions relative to an airport +
# runway. In order to work it needs a database of airports and runways and for
# this we lean on the FlightGear apt.dat.ws3.gz  If you need help tracking down
# a copy of this file, please ask!  But this really only needs to be run once by
# the project admin whenever the local db needs to be updated


import argparse
import gzip
from math import atan2, cos, pi, sin
import numpy as np
import pickle

parser = argparse.ArgumentParser(description="Parse apt.dat file and do stuff.")
parser.add_argument("aptdat", help="path to apt.dat.gz (or apt.dat.ws3.gz) file")
args = parser.parse_args()

apt_rwy_db = {}
apt = {}

with gzip.open(args.aptdat, "r") as f:
    in_apt = False
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
                print("Scanning:", id, name)
                in_apt = True
                apt = { "id": id, "alt_ft": alt_ft, "name": name, "rwys": [] }
            elif tokens[0] == "100":
                rwy1 = tokens[8]
                rwy2 = tokens[17]
                lat1 = float(tokens[9])
                lon1 = float(tokens[10])
                lat2 = float(tokens[18])
                lon2 = float(tokens[19])
                rwy = { "rwy1": rwy1, "rwy2": rwy2, "lat1": lat1, "lon1": lon1, "lat2": lat2, "lon2": lon2 }
                apt["rwys"].append(rwy)
        else:
            apt_rwy_db[id] = apt

path = "apt_rwy_db.pkl"
print("saving airport and runway db:", path)
with open(path, "wb") as f:
    pickle.dump(apt_rwy_db, f)