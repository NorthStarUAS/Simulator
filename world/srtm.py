from math import floor, sqrt
from matplotlib import pyplot as plt
import numpy as np
import os
import pickle
import scipy.interpolate
from scipy import signal
import struct
import urllib.request
import zipfile

import navpy

import sys
sys.path.append("..")
from lib.constants import ft2m

# For runway leveling in the SRTM terrain
segment_length = 25
cutoff_freq = segment_length * 0.003  # bigger values == tighter fit
b, a = signal.butter(2, cutoff_freq, 'lowpass')

# return the lower left corner of the 1x1 degree tile containing
# the specified lla coordinate
def lla_ll_corner(lat_deg, lon_deg):
    return int(floor(lat_deg)), int(floor(lon_deg))

# return the tile base name for the specified coordinate
def make_tile_name(lat, lon):
    ll_lat, ll_lon = lla_ll_corner(lat, lon)
    if ll_lat < 0:
        slat = "S%02d" % -ll_lat
    else:
        slat = "N%02d" % ll_lat
    if ll_lon < 0:
        slon = "W%03d" % -ll_lon
    else:
        slon = "E%03d" % ll_lon
    return slat + slon

def gen_tile_range(se_lat, se_lon, nw_lat, nw_lon):
    lat1, lon1 = lla_ll_corner( se_lat, nw_lon )
    lat2, lon2 = lla_ll_corner( nw_lat, se_lon )
    return lat1, lon1, lat2, lon2

class SRTM():
    def __init__(self, lat, lon):
        self.lat, self.lon = lla_ll_corner(lat, lon)
        self.tilename = make_tile_name(lat, lon)
        self.raw_interp = None
        self.level_pts = None

    def load(self, filename):
        print("SRTM: loading .hgt file:", filename)
        #f = open(cache_file, "rb")
        zip = zipfile.ZipFile(filename)
        if zipfile.Path(filename, self.tilename + ".hgt").exists():
            f = zip.open(self.tilename + ".hgt", "r")
        elif zipfile.Path(filename, self.tilename.lower() + ".hgt").exists():
            f = zip.open(self.tilename.lower() + ".hgt", "r")
        else:
            print("cannot find the right entry in this zip file!")
            f = None
        contents = f.read()
        f.close()
        # read 1,442,401 (1201x1201) high-endian
        # signed 16-bit words into self.z
        srtm_z = struct.unpack(">1442401H", contents)
        if srtm_z is None:
            print(self.lat, self.lon, self.tilename)
            quit()
        if True:
            # (fast w/ numpy functions) convert to our grid interpolator order
            self.raw_pts = np.array(srtm_z)
            self.raw_pts = self.raw_pts.reshape((1201,1201))
            self.raw_pts = np.flipud(self.raw_pts).T
            # sanity filter height values
            self.raw_pts[self.raw_pts>10000] = 0
            self.raw_pts[self.raw_pts<0] = 0
        if False:
            # (slow) long/manual python nested iterative version
            self.raw_pts = np.zeros((1201, 1201))
            for r in range(0,1201):
                for c in range(0,1201):
                    idx = (1201*r)+c
                    va = srtm_z[idx]
                    if va == 65535 or va < 0 or va > 10000:
                        va = 0.0
                    z = va
                    self.raw_pts[c,1200-r] = z

    def make_raw_interpolator(self):
        print("SRTM: constructing RAW interpolator")
        self.x = np.linspace(self.lon, self.lon+1, 1201)
        self.y = np.linspace(self.lat, self.lat+1, 1201)
        self.raw_interp = scipy.interpolate.RegularGridInterpolator((self.x, self.y), self.raw_pts, bounds_error=False, fill_value=-32768)
        #print self.raw_interp([-93.14530573529404, 45.220697421008396])
        #for i in range(20):
        #    x = -94.0 + random.random()
        #    y = 45 + random.random()
        #    z = self.raw_interp([x,y])
        #    print [x, y, z[0]]

        # print("is close:", np.isclose(raw_pts, raw_pts))
        # print("t1:", raw_pts)
        # print("raw_pts:", raw_pts)

    def raw_interpolate(self, point_list):
        if self.raw_interp is None:
            self.make_raw_interpolator()
        return self.raw_interp(point_list)

    def make_interpolator(self):
        print("SRTM: constructing LEVEL interpolator")
        self.x = np.linspace(self.lon, self.lon+1, 1201)
        self.y = np.linspace(self.lat, self.lat+1, 1201)
        self.interp = scipy.interpolate.RegularGridInterpolator((self.x, self.y), self.level_pts, bounds_error=False, fill_value=-32768)
        #print self.raw_interp([-93.14530573529404, 45.220697421008396])
        #for i in range(20):
        #    x = -94.0 + random.random()
        #    y = 45 + random.random()
        #    z = self.raw_interp([x,y])
        #    print [x, y, z[0]]

        # print("is close:", np.isclose(raw_pts, raw_pts))
        # print("t1:", raw_pts)
        # print("raw_pts:", raw_pts)

    def interpolate(self, point_list):
        if self.interp is None:
            self.make_interpolator()
        return self.interp(point_list)

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

class Cache():
    def __init__(self, srtm_dir, download=True):
        self.cache_dir = srtm_dir
        self.cache = {}
        self.do_plot = False
        self.do_download = download

        filename = "data/airports/srtm_runways.pkl"
        print("Loading list of runways sorted by tile:", filename)
        self.by_tiles = {}
        if os.path.exists(filename):
            with open(filename, "rb") as f:
                self.by_tiles = pickle.load(f)
        else:
            print("  No runway info file found!")

    def ensure_tile_downloaded(self, tilename):
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
                    return None
        if not os.path.exists(cache_file) and self.do_download:
            url = "https://bailu.ch/dem3/" + tilename[:3] + "/" + filename
            download_target = self.cache_dir + '/' + filename
            print("SRTM: downloading:", url)
            file = urllib.request.URLopener()
            try:
                print(file.retrieve(url, download_target))
            except:
                print("Cannot download srtm tile:", tilename)
        if os.path.exists(cache_file):
            return os.path.join(self.cache_dir, filename)
        else:
            return None

    def load_tile(self, lat, lon):
        tilename = make_tile_name(lat, lon)
        if tilename not in self.cache:
            filename = self.ensure_tile_downloaded(tilename)
            tile = SRTM(lat, lon)
            if filename is not None:
                tile.load(filename)
                if self.do_plot:
                    tile.plot_raw()
                self.cache[tilename] = tile
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

    # Runway Leveling (semi higher level (may need to look outside current tile),
    # semi lower level function adjusting tile elevations)
    def level_runways(self, tilename):
        if tilename not in self.cache:
            return

        srtm_tile = self.cache[tilename]
        if srtm_tile.level_pts is not None:
            # already complete
            return

        if tilename in self.by_tiles:
            runways = self.by_tiles[tilename]
        else:
            runways = []
        work_mask = np.zeros((1201, 1201))
        srtm_tile.level_pts = np.copy(srtm_tile.raw_pts)
        print("here0:", srtm_tile.level_pts, srtm_tile.raw_pts )
        dist_array = np.ones((1201, 1201)) * 10000
        for runway in runways:
            # 1. leverage lla -> ned coordinate transformation to get real world length in meters
            alt_ft = float(runway[1])
            alt_m = alt_ft * ft2m
            w = float(runway[1])
            w2 = w * 0.5
            lla1 = [float(runway[9]), float(runway[10]), alt_m]
            lla2 = [float(runway[18]), float(runway[19]), alt_m]
            ned1 = navpy.lla2ned(lla1[0], lla1[1], lla1[2], lla1[0], lla1[1], lla1[2])
            ned2 = navpy.lla2ned(lla2[0], lla2[1], lla2[2], lla1[0], lla1[1], lla1[2])
            print("neds:", ned1, ned2)
            l = sqrt( (ned2[1] - ned1[1])**2 + (ned2[0] - ned1[0])**2 )
            print("runway:", ned1, ned2, l)

            # 2. divide up the runway length in some reasonable # of segments
            divs = int(l / segment_length) + 1
            print("divs:", divs)
            lat_step = (lla2[0] - lla1[0]) / divs
            lon_step = (lla2[1] - lla1[1]) / divs
            print(lat_step, lon_step)
            fit_pts = []
            fit_vals = []
            xm = []
            print(lla1[0], lla2[0] + lat_step, lat_step)
            print(lla1[1], lla2[1] + lon_step, lon_step)
            lat = lla1[0]
            lon = lla1[1]
            for i in range(divs+1):
                # print("ll:", lat, lon)
                fit_pts.append([lon, lat])
                fit_vals.append(None)
                xm.append(i*segment_length)
                lat += lat_step
                lon += lon_step

            # 3. raw interpolate the elevation along the centerline at each of these
            #    subdivided points (from original srtm data)
            fit_pts = np.array(fit_pts)
            # print(fit_pts)
            lat_min = np.min(fit_pts[:,1])
            lat_max = np.max(fit_pts[:,1])
            lon_min = np.min(fit_pts[:,0])
            lon_max = np.max(fit_pts[:,0])
            lat1, lon1, lat2, lon2 = gen_tile_range(lat_min, lon_max, lat_max, lon_min)
            print("lla range:", lat_min, lon_max, lat_max, lon_min)
            print("srtm tile range:", lat1, lon1, lat2, lon2)

            # for each srtm tile this region spans, interpolate as many elevation
            # values as we can, then copy the good values into zs.  When we finish
            # all the listed tiles, we should have found elevations for the entire
            # set of points.
            for lat in range(lat1, lat2+1):
                for lon in range(lon1, lon2+1):
                    raw_tile = self.get_tile(lat, lon)
                    print("raw_tile:", raw_tile)
                    if raw_tile is not None:
                        zs = raw_tile.raw_interpolate(np.array(fit_pts))
                        #print zs
                        # copy the good altitudes back to the corresponding ned points
                        if len(zs) == len(fit_pts):
                            for i in range(len(fit_pts)):
                                if zs[i] > -10000:
                                    fit_vals[i] = zs[i]

            for i in range(len(fit_vals)):
                if fit_vals[i] is None:
                    print("Problem interpolating elevation for:", fit_pts[i], "(ocean?)")
                    fit_vals[i] = 0.0

            # 4. scipy.filtfilt() with suitable bandwidth values to get a reasonable smoothed surface.
            if len(fit_vals) > 10:
                # print("fit_vals:", len(fit_vals), fit_vals)
                elev_fit = signal.filtfilt(b, a, fit_vals)

                # plt.figure()
                # plt.plot(xm, fit_vals, ".")
                # plt.plot(xm, elev_fit)
                # # plt.axis("equal")
                # plt.xlabel("length (m)")
                # plt.ylabel("elev (m)")
                # plt.title(id)
                # plt.show()
            else:
                elev_fit = fit_vals

            # update the work mask with cells that potentially need updating
            print("here:", srtm_tile, srtm_tile.level_pts, srtm_tile.raw_pts)
            for i in range(len(fit_pts)):
                [lon, lat] = fit_pts[i]
                elev = elev_fit[i]
                xidx = int(round((lon - srtm_tile.lon) * 1200))
                yidx = int(round((lat - srtm_tile.lat) * 1200))
                # print(lon, lat, xidx, yidx)
                ned1 = np.array([0, 0, 0])
                # ned1 = navpy.lla2ned(lla1[0], lla1[1], lla1[2], lla1[0], lla1[1], lla1[2])
                for r in range(yidx-2, yidx+3):
                    if r < 0 or r > 1200:
                        continue
                    for c in range(xidx-2, xidx+3):
                        if c < 0 or c > 1200:
                            continue
                        work_mask[r,c] = 1
                        grid_lat = srtm_tile.lat + r / 1200
                        grid_lon = srtm_tile.lon + c / 1200
                        ned2 = navpy.lla2ned(grid_lat, grid_lon, 0.0, lat, lon, 0.0)
                        dist = np.linalg.norm(ned2 - ned1)
                        # print("dist:", dist)
                        if dist < dist_array[r,c]:
                            dist_array[r,c] = dist
                            diff = elev - srtm_tile.raw_pts[c,r]
                            # print("rwy fit:", elev, "raw:", srtm_tile.raw_pts[c,r], "diff:", diff)
                            min = 100
                            blend = 100
                            if dist < min:
                                val = elev
                                # print("  using rwy elev:", val)
                            elif dist < min + blend:
                                val = diff * (dist - min) / blend + srtm_tile.raw_pts[c,r]
                                # print("  using blended elev:", diff, val)
                            else:
                                val = srtm_tile.raw_pts[c,r]
                                # print("  using raw elev:", val)
                            srtm_tile.level_pts[c,r] = val

        # 5. ...profit! somehow aka traverse nearby srtm posts and adjust
        #               elevation based on nearness to the fit line ...

        # I could do an n x m brute force comparison of all the points ... but I don't want to.
        # can I use an opencv style mask somehow?

        # 5.1 for each runway, make a list of SRTM points that are in range (above)
        # 5.2 for each of these SRTM points, find closest rwy point
        # 5.3 new SRTM elevation is func(nearest_apt_pt_elev, distance, original elevation)

        if False:
            work_idx = (dist_array<10000).nonzero()
            print("work idx:", work_idx)
            for i in range(len(work_idx[0])):
                r = work_idx[0][i]
                c = work_idx[1][i]
                # yes raw_pts is c,r but other stuff is r,c
                srtm_tile.raw_pts[c,r] = srtm_tile.level_pts[c,r]
                # lat = srtm_tile.lat + r / 1200
                # lon = srtm_tile.lon + c / 1200
                # print("work:", lat, lon)

        # remake the interpolator with leveled elevation points
        srtm_tile.make_interpolator()
        # plt.figure()
        # plt.imshow(srtm_tile.level_pts.T, origin="lower")
        # plt.figure()
        # plt.imshow(work_mask, origin="lower")
        # plt.show()

if __name__ == "__main__":
    import pathlib
    srtm_dir = os.path.join(pathlib.Path.home(), ".scenery_viewer", "cache", "srtm")
    pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)

    srtm_cache = Cache(srtm_dir)

    lat = 46.5
    lon = -92.2
    tile = srtm_cache.get_tile(lat, lon)

    # filename = "../data/airports/srtm_runways.pkl"
    # print("Loading list of runways sorted by tile:", filename)
    # by_tiles = {}
    # if os.path.exists(filename):
    #     with open(filename, "rb") as f:
    #         by_tiles = pickle.load(f)

    tilename = make_tile_name(lat, lon)
    print(tilename)
    srtm_cache.level_runways(tilename)