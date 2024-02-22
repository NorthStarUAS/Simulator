from math import floor
from matplotlib import pyplot as plt
import numpy as np
import os
import pickle
import rasterio, rasterio.plot
import scipy.interpolate
import scipy.ndimage
from tqdm import tqdm

from scipy import signal
import struct
import urllib.request
import zipfile

import navpy

FABDEM_VERSION = "FABDEM_V1-2"

# For runway leveling in the SRTM terrain
segment_length = 25
cutoff_freq = segment_length * 0.001  # bigger values == tighter fit
b, a = signal.butter(2, cutoff_freq, 'lowpass')

# return the lower left corner of the 1x1 degree tile containing
# the specified lla coordinate
def lla_ll_tens_corner(lat_deg, lon_deg):
    return int(floor(lat_deg/10))*10, int(floor(lon_deg/10))*10

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

# return the tile base name for the specified coordinate
def make_zip_name(lat, lon):
    ll_lat, ll_lon = lla_ll_tens_corner(lat, lon)
    ur_lat = ll_lat + 10
    ur_lon = ll_lon + 10
    base = make_lat_part(ll_lat) + make_lon_part(ll_lon) + "-" + make_lat_part(ur_lat) + make_lon_part(ur_lon)
    return base + "_" + FABDEM_VERSION + ".zip"

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

class FABDEM():
    def __init__(self, lat, lon):
        self.lat, self.lon = lla_ll_corner(lat, lon)
        self.zipname = make_zip_name(lat, lon)
        self.tilename = make_tile_name(lat, lon)
        self.base_interp = None
        self.delaunay_interp = None
        self.smooth_patches = None

    def load(self, dirname):
        zippath = os.path.join(dirname, self.zipname)
        tilepath = self.tilename + "_" + FABDEM_VERSION + ".tif"
        print("FABDEM: from", zippath)
        print("FABDEM: loading", tilepath)
        path = "zip+file://" + zippath + "!" + tilepath
        self.contents = rasterio.open(path)
        print(self.contents.count, self.contents.height, self.contents.width)

        # rasterio.plot.show(contents)
        # self.raw_pts = self.contents.read(1)

        if False:
            print(self.contents.xy(0,0))
            print(self.contents.xy(3600,3600))
            for val in self.contents.sample([(-92.2523293572,46.52010289357)]):
                print(val)

        # self.contents.close()
        # quit()

        # print(self.raw_pts.shape)
        # plt.imshow(self.raw_pts)
        # plt.show()
        # quit()

    # # works but is really slow to build :-(
    # def make_delaunay_interpolator(self):
    #     print("SRTM: constructing DELAUNAY interpolator")
    #     x = np.linspace(self.lon, self.lon+1, 1201)
    #     y = np.linspace(self.lat, self.lat+1, 1201)
    #     print("x:", x)
    #     print("y:", y)
    #     xs, ys = np.meshgrid(x, y)
    #     print("xs:", xs.shape, xs)
    #     print("ys:", ys.shape, ys)
    #     xys = np.dstack((xs, ys))
    #     # xys = np.array(list(zip(np.meshgrid(x, y)))).reshape((-1,2))
    #     # print("xys (list):", list(zip(np.meshgrid(x, y))))
    #     print(type(xys), type(self.raw_pts))
    #     print(xys.shape, self.raw_pts.reshape(-1).shape)
    #     print("xys:", xys.reshape(-1,2))
    #     self.delaunay_interp = scipy.interpolate.LinearNDInterpolator(xys.reshape(-1,2), self.raw_pts.reshape(-1))

    # def delaunay_interpolate(self, point_list):
    #     if self.delaunay_interp is None:
    #         self.make_delaunay_interpolator()
    #     return self.delaunay_interp(point_list)

    # def make_base_interpolator(self):
    #     rows, cols = self.raw_pts.shape
    #     print("FABDEM: constructing interpolator", rows, "x", cols)
    #     x = np.linspace(self.lon, self.lon+1, 1201)
    #     y = np.linspace(self.lat, self.lat+1, 1201)
    #     self.base_interp = scipy.interpolate.RegularGridInterpolator((x, y), self.raw_pts, bounds_error=False, fill_value=-32768)
    #     #print self.raw_interp([-93.14530573529404, 45.220697421008396])
    #     #for i in range(20):
    #     #    x = -94.0 + random.random()
    #     #    y = 45 + random.random()
    #     #    z = self.raw_interp([x,y])
    #     #    print [x, y, z[0]]

    #     # print("is close:", np.isclose(raw_pts, raw_pts))
    #     # print("t1:", raw_pts)
    #     # print("raw_pts:", raw_pts)

    def base_interpolate(self, coords):
        # print(coords[:,:2].shape)
        vals = self.contents.sample(coords)
        # copy the good altitudes back to the corresponding ned points
        # tmp = []
        for i, val in enumerate(vals):
            # tmp.append(float(val))
            if val > -9998:
                coords[i][2] = float(val)
        # print("tmp:", tmp)

    def full_interpolate(self, coords):
        self.base_interpolate(coords)
        for patch in self.smooth_patches:
            patch.lla_interpolate(coords)

    def plot_raw(self):
        zzz = np.zeros((1201,1201))
        for r in range(0,1201):
            for c in range(0,1201):
                va=srtm_z[(1201*r)+c]
                if (va==65535 or va<0 or va>10000):
                    va=0.0
                zzz[r][c]=float(va)

        #zz=np.log1p(zzz)
        imshow(zzz, interpolation='bilinear',cmap=cm.gray,alpha=1.0)
        grid(False)
        show()

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

    def ensure_zip_downloaded(self, zipname):
        cache_file = os.path.join(self.cache_dir, zipname)
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
            url = "https://data.bris.ac.uk/datasets/s5hqmjcdj8yo2ibzi9b4ew3sn/" + zipname
            download_target = self.cache_dir + '/' + zipname
            print("FABDEM: downloading:", url)
            try:
                urllib.request.urlretrieve(url, download_target, show_progress)
            except:
                print("Cannot download FABDEM 10x10 zip:", zipname)
        if os.path.exists(cache_file):
            return True
        else:
            return False

    def load_tile(self, lat, lon):
        tile = FABDEM(lat, lon)
        if tile.tilename not in self.cache:
            if self.ensure_zip_downloaded(tile.zipname):
                tile.load(self.cache_dir)
                if self.do_plot:
                    tile.plot_raw()
                self.cache[tile.tilename] = tile
                return True
        return False

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
    def __init__(self, fabdem_cache, lat_min, lat_max, lon_min, lon_max, nedref, step_size=25):
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
        # coords = np.array(coords)
        # print(coords.shape)
        lat1, lon1, lat2, lon2 = gen_tile_range(lat_min, lon_max, lat_max, lon_min)
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                fabdem_tile = fabdem_cache.get_tile(lat, lon)
                # tilename = srtm.make_tile_name(lat, lon)
                # srtm_cache.level_runways(tilename) # if needed
                if fabdem_tile is not None:
                    fabdem_tile.base_interpolate(coords)
        # print("raw vals:", vals)
        raw_vals = np.array(coords)[:,2].reshape( (edivs+1, ndivs+1)).T
        print("raw_vals reshape:", raw_vals)

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
    fabdem_dir = os.path.join(pathlib.Path.home(), ".nsWorld", "cache", "fabdem")
    pathlib.Path(fabdem_dir).mkdir(parents=True, exist_ok=True)

    fabdem_cache = DEMCache(fabdem_dir)

    # lat = 46.5
    # lon = -92.2
    lat = 37.2
    lon = -113.5
    tile = fabdem_cache.get_tile(lat, lon)

    # print(tile.delaunay_interpolate( [-92.25, 46.45 ] ))

    # filename = "../data/airports/srtm_runways.pkl"
    # print("Loading list of runways sorted by tile:", filename)
    # by_tiles = {}
    # if os.path.exists(filename):
    #     with open(filename, "rb") as f:
    #         by_tiles = pickle.load(f)

    tilename = make_tile_name(lat, lon)
    print(tilename)

    coords = [ [-92.25, 46.45, 0 ], [-92.20, 46.84, 0] ]
    for i, val in enumerate(tile.contents.sample(coords)):
        print("coord:", coords[i], val)

    fabdem_cache.make_smooth_patches(tilename)

    tile.full_interpolate( coords )
    print("coords:", coords)
