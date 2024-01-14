from math import floor, sqrt
from matplotlib import pyplot as plt
import numpy as np
import os
import pickle
import scipy.interpolate
import scipy.ndimage

from scipy import signal
import struct
import urllib.request
import zipfile

import navpy

# For runway leveling in the SRTM terrain
segment_length = 25
cutoff_freq = segment_length * 0.001  # bigger values == tighter fit
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
        self.delaunay_interp = None
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

    # works but is really slow to build :-(
    def make_delaunay_interpolator(self):
        print("SRTM: constructing DELAUNAY interpolator")
        x = np.linspace(self.lon, self.lon+1, 1201)
        y = np.linspace(self.lat, self.lat+1, 1201)
        print("x:", x)
        print("y:", y)
        xs, ys = np.meshgrid(x, y)
        print("xs:", xs.shape, xs)
        print("ys:", ys.shape, ys)
        xys = np.dstack((xs, ys))
        # xys = np.array(list(zip(np.meshgrid(x, y)))).reshape((-1,2))
        # print("xys (list):", list(zip(np.meshgrid(x, y))))
        print(type(xys), type(self.raw_pts))
        print(xys.shape, self.raw_pts.reshape(-1).shape)
        print("xys:", xys.reshape(-1,2))
        self.delaunay_interp = scipy.interpolate.LinearNDInterpolator(xys.reshape(-1,2), self.raw_pts.reshape(-1))

    def delaunay_interpolate(self, point_list):
        if self.delaunay_interp is None:
            self.make_delaunay_interpolator()
        return self.delaunay_interp(point_list)

    def make_raw_interpolator(self):
        print("SRTM: constructing RAW interpolator")
        x = np.linspace(self.lon, self.lon+1, 1201)
        y = np.linspace(self.lat, self.lat+1, 1201)
        self.raw_interp = scipy.interpolate.RegularGridInterpolator((x, y), self.raw_pts, bounds_error=False, fill_value=-32768)
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
        x = np.linspace(self.lon, self.lon+1, 1201)
        y = np.linspace(self.lat, self.lat+1, 1201)
        self.interp = scipy.interpolate.RegularGridInterpolator((x, y), self.level_pts, bounds_error=False, fill_value=-32768)
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

        basepath = os.path.dirname(os.path.realpath(__file__))
        filename = os.path.join(basepath, "data/airports/smooth_patches.pkl")
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

    # Airport area Leveling (semi higher level (may need to look outside current tile),
    # semi lower level function adjusting tile elevations)
    def level_airports(self, tilename):
        if tilename not in self.cache:
            return

        srtm_tile = self.cache[tilename]
        if srtm_tile.level_pts is not None:
            # already complete
            return

        if tilename in self.by_tiles:
            airports = self.by_tiles[tilename]
        else:
            airports = []

        srtm_tile.level_pts = np.copy(srtm_tile.raw_pts)
        # print("here0:", srtm_tile.level_pts, srtm_tile.raw_pts )
        dist_array = np.ones((1201, 1201)) * 10000

        for lat_min, lat_max, lon_min, lon_max in airports:
            print("smooth patch:", lat_min, lat_max, lon_min, lon_max)
            nedref = [0.5*(lat_min+lat_max), 0.5*(lon_min+lon_max), 0]
            patch = SmoothPatch(self, lat_min, lat_max, lon_min, lon_max, nedref)

            xmin = int((lon_min - srtm_tile.lon) * 1200)
            xmax = int((lon_max - srtm_tile.lon) * 1200) + 1
            ymin = int((lat_min - srtm_tile.lat) * 1200)
            ymax = int((lat_max - srtm_tile.lat) * 1200) + 1
            if xmin < 0: xmin = 0
            if xmax > 1200: xmax = 1200
            if ymin < 0: ymin = 0
            if ymax > 1200: ymax = 1200
            for r in range(ymin, ymax+1):
                for c in range(xmin, xmax+1):
                    lla = [ srtm_tile.lon + c/1200, srtm_tile.lat + r/1200, 0 ]
                    result = patch.lla_interpolate([lla])
                    if lla[2] > -998:
                        # print("patch update:", r, c, lla, srtm_tile.raw_pts[c,r], "->", lla[2])
                        srtm_tile.level_pts[c,r] = lla[2]
                    # else:
                    #     print("patch no-update:", r, c, lla, srtm_tile.raw_pts[c,r], "->", lla[2])

        # remake the interpolator with leveled elevation points
        srtm_tile.make_interpolator()

        # plt.figure()
        # plt.imshow(srtm_tile.raw_pts.T, origin="lower")
        # plt.colorbar()
        # plt.figure()
        # plt.imshow(srtm_tile.level_pts.T, origin="lower")
        # plt.colorbar()
        # plt.show()


    # Runway Leveling (semi higher level (may need to look outside current tile),
    # semi lower level function adjusting tile elevations)
    def old_level_runways(self, tilename):
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
        # print("here0:", srtm_tile.level_pts, srtm_tile.raw_pts )
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
                    # print("raw_tile:", raw_tile)
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
            # print("here:", srtm_tile, srtm_tile.level_pts, srtm_tile.raw_pts)
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

        # sample the airport coverate area at high density
        fit_pts = []
        for lat in np.linspace(lat_min, lat_max, ndivs+1):
            for lon in np.linspace(lon_min, lon_max, edivs+1):
                fit_pts.append( (lon, lat) )
        vals = [ nedref[2] ] * len(fit_pts)
        lat1, lon1, lat2, lon2 = gen_tile_range(lat_min, lon_max, lat_max, lon_min)
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                srtm_tile = srtm_cache.get_tile(lat, lon)
                # tilename = srtm.make_tile_name(lat, lon)
                # srtm_cache.level_runways(tilename) # if needed
                if srtm_tile is not None:
                    zs = srtm_tile.raw_interpolate(np.array(fit_pts))
                    #print zs
                    # copy the good altitudes back to the corresponding ned points
                    if len(zs) == len(fit_pts):
                        for i in range(len(fit_pts)):
                            if zs[i] > -10000:
                                # llas[i][2] = zs[i]
                                vals[i] = zs[i]
        # print("raw vals:", vals)
        raw_vals = np.array(vals).reshape( (ndivs+1, edivs+1))
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

    def lla_interpolate(self, llas):
        pts = []
        for lla in llas:
            pts.append( lla[:2] )
        vals = self.lla_interp(pts)
        for i in range(len(llas)):
            llas[i][2] = vals[i]

if __name__ == "__main__":
    import pathlib
    srtm_dir = os.path.join(pathlib.Path.home(), ".nsWorld", "cache", "srtm")
    pathlib.Path(srtm_dir).mkdir(parents=True, exist_ok=True)

    srtm_cache = Cache(srtm_dir)

    lat = 46.5
    lon = -92.2
    tile = srtm_cache.get_tile(lat, lon)

    # print(tile.delaunay_interpolate( [-92.25, 46.45 ] ))

    # filename = "../data/airports/srtm_runways.pkl"
    # print("Loading list of runways sorted by tile:", filename)
    # by_tiles = {}
    # if os.path.exists(filename):
    #     with open(filename, "rb") as f:
    #         by_tiles = pickle.load(f)

    tilename = make_tile_name(lat, lon)
    print(tilename)
    srtm_cache.level_airports(tilename)