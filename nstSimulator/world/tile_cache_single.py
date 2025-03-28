# disk cache for tiles (fixme: rename to tile_cache.py)

import os
from pathlib import Path
from pygeotile.tile import Tile  # pip install pygeotile
import http.client
import time
import traceback

from panda3d.core import *

# Fetch and cache slippy tiles using http.client, returns the tile as either
# a pnm image or tex.

class SlippyCache():
    def __init__(self, rootpath, url=None, root=None, ext=".png", options="", index_scheme="slippy"):
        self.rootpath = rootpath
        self.url = url
        self.root = root
        self.ext = ext
        self.options = options
        self.index_scheme = index_scheme
        self.connect()

    def connect(self):
        if self.index_scheme == "slippy":
            self.conn = http.client.HTTPSConnection(self.url, timeout=10)
        elif self.index_scheme == "quadkey" or self.index_scheme == "google":
            self.conn = http.client.HTTPConnection(self.url, timeout=10)

    def ensure_path_in_cache(self, level, x):
        path = os.path.join(self.rootpath, "%d" % level, "%d" % x)
        if not os.path.exists(path):
            Path(path).mkdir(parents=True, exist_ok=True)
        return path

    def download_tile(self, path, file, request):
        print("fetching:", self.url, path, "to", file)
        Path(path).mkdir(parents=True, exist_ok=True)
        success = False
        while not success:
            try:
                if self.index_scheme == "google":
                    headers = {"User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/119.0"}
                    self.conn.request("GET", request, headers=headers)
                else:
                    self.conn.request("GET", request)
                response = self.conn.getresponse()
                print(response.status, response.reason)
                data = response.read()  # This will return entire content.
                with open(file, "wb") as f:
                    f.write(data)
                success = True
            except Exception:
                print(traceback.format_exc())
                # or
                #print(sys.exc_info()[2])
                print("retrying in one second...")
                time.sleep(1)
                self.connect()

    def ensure_tile_in_cache(self, level, x, y):
        path = self.ensure_path_in_cache(level, x)
        file = os.path.join(path, "%d" % y + self.ext)

        # check if file exists and has non-zero length
        if os.path.exists(file):
            file_stats = os.stat(file)
            if file_stats.st_size > 0:
                return file

        # file needs to be fetched
        if self.index_scheme == "slippy":
            request = self.root + "/%d/%d/%d" % (level, x, y) + self.ext + self.options
        elif self.index_scheme == "quadkey":
            tms_y = (2 ** level) - y - 1
            tile = Tile.from_tms(tms_x=x, tms_y=tms_y, zoom=level)
            print("quadkey:", tile.quad_tree)
            request = self.root.format(tile.quad_tree) + self.ext + self.options
            print("request:", request)
        elif self.index_scheme == "google":
            tms_y = (2 ** level) - y - 1
            tile = Tile.from_tms(tms_x=x, tms_y=tms_y, zoom=level)
            print("google:", tile.google)
            x, y = tile.google
            request = self.root.format(x, y, level)
            print("request:", request)
        self.download_tile(path, file, request)
        return file

    def get_tile_as_pnm(self, level, x, y):
        file = self.ensure_tile_in_cache(level, x, y)
        with open(file, "rb") as f:
            data = f.read()
            p = PNMImage()
            p.read(StringStream(data))
            return p

    def get_tile_as_tex(self, level, x, y):
        p = self.get_tile_as_pnm(level, x, y)
        tex = Texture()
        tex.load(p)
        return tex

# https://github.com/tilezen/joerd/tree/master/docs
# https://s3.amazonaws.com/elevation-tiles-prod/{terrarium,normal}/8/62/90.png

# test
if __name__ == "__main__" and __package__ is None:
    dc = SlippyCache("./testcache", "https://tile.openstreetmap.org")
    dc.get_tile_as_tex(8, 62, 91)
