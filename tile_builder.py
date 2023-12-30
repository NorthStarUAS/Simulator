#!/usr/bin/env python3

import errno
import sys

from panda3d.core import *

from world import builder

loadPrcFileData("", "compressed-textures 1") # compress textures when we load/save them

# tile = gen_tile.gen_terrain_node(7, 30, 40, "google")

builder = builder.Builder(".scenery_viewer")

while True:
    line = input()
    # print("got:", line.split(","))
    (zoom, x, y, style) = line.split(",")
    tile = builder.gen_terrain_node(int(zoom), int(x), int(y), style)
    print("complete")
    sys.stdout.flush()
