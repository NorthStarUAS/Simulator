import json
import os
from pathlib import Path

from panda3d.core import *
import navpy

from . import slippy_tiles

apt_zoom_level = 9

class apt_mgr:
    def __init__(self, dot_root):
        self.apt_cache = { "children": {} }
        self.dot_root = dot_root

    def init_airports(self, lat_deg, lon_deg, vis_range_m):
        # airports
        tiles = slippy_tiles.get_tiles_in_range(lat_deg, lon_deg, apt_zoom_level, vis_range_m)
        print("airport tiles:", tiles)
        new_nodes = []
        for tile in tiles:
            # (zoom, x, y) = tile
            # tile_node = NodePath( "%d/%d/%d" % (zoom, x, y) )
            # _, _, _, _, center_lat, center_lon = slippy_tiles.get_tile_size(x, y, zoom)
            # new = { "name": "%d/%d/%d" % (zoom, x, y),
            #         "index": (zoom, x, y),
            #         "node": tile_node,
            #         "center_lla": [center_lat, center_lon, 0.0],
            #         "children": {},
            #         }
            # self.apt_cache["children"][new["name"]] = new
            airport_dir = os.path.join(Path.home(), self.dot_root, "cache", "airports")
            path = os.path.join(airport_dir, "%d" % tile[0], "%d" % tile[1], "%d" % tile[2])
            print("airports path:", path)
            if os.path.exists(path):
                for child in Path(path).iterdir():
                    if str(child).endswith(".json"):
                        (root, ext) = os.path.splitext(child.name)
                        print(child.name, root, ext)
                        f = open(os.path.join(path, child.name), "r")
                        info = json.load(f)
                        bam_name = os.path.join(path, root + ".bam")
                        if os.path.exists(bam_name):
                            apt_node = NodePath( path )
                            # apt = genapt.genapt(os.path.join(path, bam_name))
                            apt_node = loader.loadModel(bam_name)
                            new = { "name": root,
                                "index": None,
                                "node": apt_node,
                                "center_lla": info["nedref"],
                                "children": {},
                            }
                            new_nodes.append(new)
                            self.apt_cache["children"][root] = new
        return new_nodes

    def update_airport_cache_pos(self, nedref):
        print("Update airport cache positions:")
        for key in self.apt_cache["children"].keys():
            tile = self.apt_cache["children"][key]
            if not tile["node"].isHidden():
                # fixme: use externally provided nedref? not the locally updated one incase out of sync?
                tile_pos = navpy.lla2ned(tile["center_lla"][0], tile["center_lla"][1], tile["center_lla"][2],
                                            nedref[0], nedref[1], nedref[2])
                print("  airport tile:", tile)
                print("  tile node:", tile["node"])
                tile["node"].setPos(tile_pos[1], tile_pos[0], -tile_pos[2])
                tile["node"].setHpr(0, nedref[0] - tile["center_lla"][0], tile["center_lla"][1] - nedref[1])

    def update(self):
        pass