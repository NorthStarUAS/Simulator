# disk cache for tiles (fixme: rename to tile_cache.py)

import asyncio
import aiohttp
import os
from pathlib import Path
from pygeotile.tile import Tile  # pip install pygeotile
import time
import traceback

from panda3d.core import *

# Fetch and cache slippy tiles using http.client, returns the tile as either
# a pnm image or tex.

async def get(session: aiohttp.ClientSession, url, file):
    print("url:", type(url), url)
    try:
        async with session.get(url) as response:
            data = await response.read()
            print("read:", len(data))
            with open(file, "wb") as f:
                f.write(data)
    except:
        print("failed ...")

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
        if self.url == "":
            return
        if self.index_scheme == "slippy" or self.index_scheme == "quadkey":
            self.session = aiohttp.ClientSession(base_url=self.url, timeout=aiohttp.ClientTimeout(total=10))
        elif self.index_scheme == "quadkey" or self.index_scheme == "google":
            self.session = aiohttp.ClientSession(base_url=self.url, timeout=aiohttp.ClientTimeout(total=10), headers={"User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/119.0"})

    def ensure_path_in_cache(self, level, x):
        path = os.path.join(self.rootpath, "%d" % level, "%d" % x)
        if not os.path.exists(path):
            Path(path).mkdir(parents=True, exist_ok=True)
        return path

    async def download_tiles(self, path, files, requests):
        print("fetching:", self.url, requests, "to", files)
        Path(path).mkdir(parents=True, exist_ok=True)
        success = False
        while not success:
            try:
                # async with aiohttp.ClientSession() as session:
                await asyncio.gather(
                    *[get(self.session, request, file) for request, file in zip(requests, files)]
                )
                # if self.index_scheme == "google":
                #     headers = {"User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/119.0"}
                #     self.conn.request("GET", request, headers=headers)
                # else:
                #     self.conn.request("GET", request)
                # response = self.conn.getresponse()
                # print(response.status, response.reason)
                # data = response.read()  # This will return entire content.
                # with open(file, "wb") as f:
                #     f.write(data)
                success = True
            except Exception:
                print(traceback.format_exc())
                # or
                #print(sys.exc_info()[2])
                print("retrying in one second...")
                time.sleep(1)
                self.connect()

    def ensure_tile_in_cache(self, level, x, y, force_download=False):
        path = self.ensure_path_in_cache(level, x)
        file = os.path.join(path, "%d" % y + self.ext)

        # check if file exists and has non-zero length
        if os.path.exists(file) and not force_download:
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

        # documented way (with errors)
        # asyncio.run(self.download_tiles(path, [file], [request]))

        # stackoverflow way
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.download_tiles(path, [file], [request]))

        return file

    def ensure_tiles_in_cache(self, level, xys, force_download=False):
        file_names = []
        files = []
        requests = []
        for i, [x, y] in enumerate(xys):
            path = self.ensure_path_in_cache(level, x)
            file = os.path.join(path, "%d" % y + self.ext)
            file_names.append( file )

            # check if file exists and has non-zero length
            file_ok = False
            if os.path.exists(file) and not force_download:
                file_stats = os.stat(file)
                if file_stats.st_size > 0:
                    file_ok = True

            if not file_ok:
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
                files.append(file)
                requests.append(request)

        if len(files):
            # print("fetching:", files, requests)

            # documented way (with errors)
            # asyncio.run(self.download_tiles(path, [file], [request]))

            # stackoverflow way
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.download_tiles(path, files, requests))

        return file_names

    # def get_tile_as_pnm(self, level, x, y):
    #     for i in range(3):
    #         # 3 tries then barf ... we escape with a return p on first success
    #         file = self.ensure_tile_in_cache(level, x, y)
    #         with open(file, "rb") as f:
    #             data = f.read()
    #             print("here0 image file length:", len(data), str(file))
    #             p = PNMImage()
    #             p.read(StringStream(data))
    #         if p.getXSize() and p.getYSize():
    #             return p
    #         else:
    #             print("Image load failed ... bad file, forcing a refetch...")
    #             file = self.ensure_tile_in_cache(level, x, y, force_download=True)

    def get_tiles_as_pnm(self, level, xys):
        pnms = [ None ] * len(xys)
        for attempt in range(3):
            # 3 attempts then barf ... we escape with a return pnms on first success
            file_names = self.ensure_tiles_in_cache(level, xys)
            print("file_names:", file_names)
            for i, file in enumerate(file_names):
                with open(file, "rb") as f:
                    data = f.read()
                    print("here0 image file length:", len(data), str(file))
                    p = PNMImage()
                    p.read(StringStream(data))
                if p.getXSize() and p.getYSize():
                    pnms[i] = p
            if None in pnms:
                print("At least one image load failed ... bad file, forcing a refetch...")
                file = self.ensure_tiles_in_cache(level, xys, force_download=True)
            else:
                return pnms
        return None

    # def get_tile_as_tex(self, level, x, y):
    #     p = self.get_tile_as_pnm(level, x, y)
    #     tex = Texture()
    #     tex.load(p)
    #     return tex

# https://github.com/tilezen/joerd/tree/master/docs
# https://s3.amazonaws.com/elevation-tiles-prod/{terrarium,normal}/8/62/90.png

# test
if __name__ == "__main__" and __package__ is None:
    dc = SlippyCache("./testcache", "https://tile.openstreetmap.org")
    dc.get_tile_as_tex(8, 62, 91)
