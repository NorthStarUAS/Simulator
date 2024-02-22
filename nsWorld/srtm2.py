from math import floor
from matplotlib import pyplot as plt
import numpy as np
import os
import pickle
import rasterio, rasterio.plot
import scipy.interpolate
import scipy.ndimage
from tqdm import tqdm

import urllib.request
import zipfile

import navpy

def make_lat_part(val):
    if val < 0:
        return "S%02d" % -val
    else:
        return "N%02d" % val

def make_lon_part(val):
    if val < 0:
        return "W%03d" % -val
    else:
        return "E%03d" % val

# return the lower left corner of the 1x1 degree tile containing
# the specified lla coordinate
def lla_ll_corner(lat_deg, lon_deg):
    return int(floor(lat_deg)), int(floor(lon_deg))

# return the tile base name for the specified coordinate
def make_tile_name(lat, lon):
    ll_lat, ll_lon = lla_ll_corner(lat, lon)
    return make_lat_part(ll_lat) + make_lon_part(ll_lon)

def gen_tile_range(se_lat, se_lon, nw_lat, nw_lon):
    lat1, lon1 = lla_ll_corner( se_lat, nw_lon )
    lat2, lon2 = lla_ll_corner( nw_lat, se_lon )
    return lat1, lon1, lat2, lon2

pbar = None
def show_progress(block_num, block_size, total_size):
    global pbar
    if pbar is None:
        pbar = tqdm(total=total_size, smoothing=0.05)
    inc = block_num*block_size - pbar.n
    pbar.update(n=inc)

class SRTM2():
    def __init__(self, lat, lon):
        self.lat, self.lon = lla_ll_corner(lat, lon)
        self.tilename = make_tile_name(lat, lon)
        self.base_interp = None
        self.smooth_patches = None

    def load(self, dirname):
        zippath = os.path.join(dirname, self.tilename + ".hgt.zip")
        tilepath = self.tilename + ".hgt"
        print("SRTM: from", zippath)
        print("SRTM: loading", tilepath)
        path = "zip+file://" + zippath + "!" + tilepath
        self.contents = rasterio.open(path)
        print("rasterio:", self.contents.count, self.contents.height, self.contents.width, self.contents.crs)
        # self.plot_raw()

        if False:
            print(self.contents.xy(0,0))
            print(self.contents.xy(1200,1200))
            for val in self.contents.sample([(-92.2523293572,46.52010289357)]):
                print(val)

        self.raw_pts = self.contents.read(1)
        self.raw_pts = np.flipud(self.raw_pts).T
        self.contents.close()

    def make_base_interpolator(self):
        print("SRTM: constructing interpolator")
        x = np.linspace(self.lon, self.lon+1, 1201)
        y = np.linspace(self.lat, self.lat+1, 1201)
        self.base_interp = scipy.interpolate.RegularGridInterpolator((x, y), self.raw_pts, bounds_error=False, fill_value=-32768)
        #print self.raw_interp([-93.14530573529404, 45.220697421008396])
        #for i in range(20):
        #    x = -94.0 + random.random()
        #    y = 45 + random.random()
        #    z = self.raw_interp([x,y])
        #    print [x, y, z[0]]

        # print("is close:", np.isclose(raw_pts, raw_pts))
        # print("t1:", raw_pts)
        # print("raw_pts:", raw_pts)

    # def base_interpolate_rasterio(self, coords):
    #     # print(coords[:,:2].shape)
    #     vals = self.contents.sample(coords)
    #     # copy the good altitudes back to the corresponding ned points
    #     # tmp = []
    #     for i, val in enumerate(vals):
    #         # tmp.append(float(val))
    #         if val > -9998:
    #             coords[i][2] = float(val)
    #     # print("tmp:", tmp)
    #     print()

    def base_interpolate(self, coords):
        if self.base_interp is None:
            self.make_base_interpolator()
        # pts = []
        # for lat, lon, alt in llas:
        #     pts.append([lon, lat])
        vals = self.base_interp(coords[:,:2])
        # print("vals:", vals)
        # copy the good altitudes back to the corresponding ned points
        if len(vals) == len(coords):
            for i in range(len(coords)):
                if vals[i] > -9998:
                    coords[i][2] = vals[i]

    def full_interpolate(self, coords):
        self.base_interpolate(coords)
        for patch in self.smooth_patches:
            patch.lla_interpolate(coords)

    def plot_raw(self):
        rasterio.plot.show(self.contents)
        # if do_it_with_matplotlib:
        #     print(self.raw_pts.shape)
        #     plt.imshow(self.raw_pts)
        #     plt.show()

class DEMCache():
    def __init__(self, cache_dir, download=True):
        self.cache_dir = cache_dir
        self.cache = {}
        self.do_plot = False
        self.do_download = download

        basepath = os.path.dirname(os.path.realpath(__file__))
        filename = os.path.join(basepath, "data/airports/smooth_patches.pkl")
        print("Loading list of runways sorted by tile:", filename)
        self.by_tiles = {}
        if os.path.exists(filename):
            with open(filename, "rb") as f:
                self.by_tiles = pickle.load(f)
        else:
            print("  No runway info file found!")

    def ensure_zip_downloaded(self, tilename):
        filename = tilename + ".hgt.zip"
        cache_file = os.path.join(self.cache_dir, filename)
        if os.path.exists(cache_file):
            # test if we have a good zip file
            try:
                zip = zipfile.ZipFile(cache_file)
            except zipfile.BadZipFile:
                if self.do_download:
                    print("Removing bad zip file:", cache_file)
                    os.remove(cache_file)
                else:
                    # print("NOT removing bad zip file (probably in mid-download):", cache_file)
                    return False
        if not os.path.exists(cache_file) and self.do_download:
            url = "https://bailu.ch/dem3/" + tilename[:3] + "/" + filename
            download_target = self.cache_dir + '/' + filename
            print("SRTM: downloading:", url)
            try:
                urllib.request.urlretrieve(url, download_target, show_progress)
                pbar.close()
            except:
                print("Cannot download SRTM tile:", filename)
        if os.path.exists(cache_file):
            return True
        else:
            return False

    def load_tile(self, lat, lon):
        tile = SRTM2(lat, lon)
        if tile.tilename not in self.cache:
            if not self.ensure_zip_downloaded(tile.tilename):
                return False
            tile.load(self.cache_dir)
            if self.do_plot:
                tile.plot_raw()
            self.cache[tile.tilename] = tile
        return True

    def get_tile(self, lat, lon):
        tilename = make_tile_name(lat, lon)
        if tilename in self.cache:
            return self.cache[tilename]
        else:
            result = self.load_tile(lat, lon)
            if result:
                return self.cache[tilename]
            else:
                return None

    # Airport area Leveling (semi higher level (may need to look outside current tile),
    # semi lower level function adjusting tile elevations)
    def make_smooth_patches(self, tilename):
        if tilename not in self.cache:
            return

        srtm_tile = self.cache[tilename]
        if srtm_tile.smooth_patches is not None:
            # already complete
            return

        if tilename in self.by_tiles:
            airports = self.by_tiles[tilename]
        else:
            airports = []

        srtm_tile.smooth_patches = []

        for lat_min, lat_max, lon_min, lon_max in airports:
            print("smooth patch:", lat_min, lat_max, lon_min, lon_max)
            nedref = [0.5*(lat_min+lat_max), 0.5*(lon_min+lon_max), 0]
            patch = SmoothPatch(self, lat_min, lat_max, lon_min, lon_max, nedref)
            srtm_tile.smooth_patches.append( patch )

class SmoothPatch:
    def __init__(self, srtm_cache, lat_min, lat_max, lon_min, lon_max, nedref, step_size=25):
        do_plot = False
        if lat_max <= lat_min:
            print("  swapping lat_min/max[0]")
            tmp = lat_max
            lat_min = lat_max
            lat_max = tmp
            lat_min -= 0.000001
            lat_max += 0.000001
        if lon_max <= lon_min:
            print("  swapping lon_min/max[0]")
            tmp = lon_max
            lon_min = lon_max
            lon_max = tmp
            lon_min -= 0.000001
            lon_max += 0.000001

        nedmin = navpy.lla2ned(lat_min, lon_min, nedref[2], nedref[0], nedref[1], nedref[2])
        nedmax = navpy.lla2ned(lat_max, lon_max, nedref[2], nedref[0], nedref[1], nedref[2])
        print("nedmin:", nedmin)
        print("nedmax:", nedmax)
        if nedmax[0] < nedmin[0]:
            print("  swapping min/max[0]")
            tmp = nedmax[0]
            nedmin[0] = nedmax[0]
            nedmax[0] = tmp
        if abs(nedmin[0] - nedmax[0]) < 1:
            print("  spreading min/max[0]")
            nedmin[0] -= 25
            nedmax[0] += 25
        if nedmax[1] < nedmin[1]:
            print("  swapping min/max[1]")
            tmp = nedmax[1]
            nedmin[1] = nedmax[1]
            nedmax[1] = tmp
        if abs(nedmin[1] - nedmax[1]) < 1:
            print("  spreading min/max[1]")
            nedmin[1] -= 25
            nedmax[1] += 25
        ndivs = int((nedmax[0] - nedmin[0]) / step_size) + 1
        edivs = int((nedmax[1] - nedmin[1]) / step_size) + 1
        print("divs n/e:", ndivs, edivs)

        # sample the airport coverage area at high density
        coords = []
        for lon in np.linspace(lon_min, lon_max, edivs+1):
            for lat in np.linspace(lat_min, lat_max, ndivs+1):
                coords.append( [lon, lat, nedref[2]] )
        coords = np.array(coords)
        # print(coords.shape)
        lat1, lon1, lat2, lon2 = gen_tile_range(lat_min, lon_max, lat_max, lon_min)
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                srtm_tile = srtm_cache.get_tile(lat, lon)
                # tilename = srtm.make_tile_name(lat, lon)
                # srtm_cache.level_runways(tilename) # if needed
                if srtm_tile is not None:
                    srtm_tile.base_interpolate(coords)
        # print("raw vals:", vals)
        raw_vals = np.array(coords)[:,2].reshape( (edivs+1, ndivs+1)).T
        # print("raw_vals reshape:", raw_vals)

        self.smooth = scipy.ndimage.gaussian_filter(raw_vals, sigma=3, mode="nearest")

        if do_plot:
            plt.figure()
            plt.imshow(raw_vals, origin="lower")
            plt.title("raw")
            plt.colorbar()
            plt.figure()
            plt.imshow(self.smooth, origin="lower")
            plt.title("smooth")
            plt.colorbar()
            plt.show()

        x = np.linspace(nedmin[1], nedmax[1], edivs+1)
        y = np.linspace(nedmin[0], nedmax[0], ndivs+1)
        self.ned_interp = scipy.interpolate.RegularGridInterpolator((x, y), self.smooth.T, bounds_error=False, fill_value=None)

        x = np.linspace(lon_min, lon_max, edivs+1)
        y = np.linspace(lat_min, lat_max, ndivs+1)
        self.lla_interp = scipy.interpolate.RegularGridInterpolator((x, y), self.smooth.T, bounds_error=False, fill_value=-9999)

    # make sure the given ned coordinates are relative to the same nedref that
    # the smoothpatch was created with!
    def ned_interpolate(self, neds):
        pts = []
        for ned in neds:
            pts.append( ned[:2] )
        vals = self.ned_interp(pts)
        for i in range(len(neds)):
            neds[i][2] = vals[i]

    def lla_interpolate(self, coords):
        pts = []
        for c in coords:
            # pts.append( lla[:2] )
            pts.append( [ c[0], c[1] ] )
        vals = self.lla_interp(pts)
        # print("vals:", vals)
        for i in range(len(coords)):
            if vals[i] > -9998:
                coords[i][2] = vals[i]

if __name__ == "__main__":
    import pathlib
    srtm_dir = os.path.join(pathlib.Path.home(), ".nsWorld", "cache", "srtm")
    pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)

    srtm_cache = DEMCache(srtm_dir)

    lat = 46.5
    lon = -92.2
    # lat = 37.2
    # lon = -113.5
    tile = srtm_cache.get_tile(lat, lon)
    print("tile:", tile)

    # filename = "../data/airports/srtm_runways.pkl"
    # print("Loading list of runways sorted by tile:", filename)
    # by_tiles = {}
    # if os.path.exists(filename):
    #     with open(filename, "rb") as f:
    #         by_tiles = pickle.load(f)

    tilename = make_tile_name(lat, lon)
    print(tilename)

    coords = np.array( [ [-92.25, 46.45, 0 ], [-92.20, 46.84, 0] ] )
    tile.base_interpolate(coords)
    for coord in coords:
        print("coord:", coord)

    srtm_cache.make_smooth_patches(tilename)

    tile.full_interpolate( coords )
    print("coords:", coords)

    coords = np.array( [ [-92.216423, 46.842346, 1430.93149] ] )
    tile.base_interpolate(coords)
    print("td coords:", coords)