# threaded tile cache manager

import copy
import cv2
from direct.stdpy import threading
from math import sqrt
import numpy as np
import os
import pathlib
import pickle
import subprocess
import time

from panda3d.core import *

import navpy

from ..utils.constants import r2d

from . import apt_mgr
from . import slippy_tiles
from . import tile_cache

terrain_min_zoom_level = 9
default_max_zoom_level = 15
vis_range_m = 100000
# vis_range_m = 4000
tex_dim = 512

state_lock = threading.Lock()
protect_ownship_lla = None
protect_cam_hpr = None
protect_nedref = None
protect_cam_pos = None
protect_lla_vel = None

cache_lock = threading.Lock()

queue_lock = threading.Lock()
reparent_queue = []
hide_queue = []
prune_queue = []

# called from outside the thread so need to lock (passed as individual values,
# not structures to make extra clear we are doing copy, not pointers)
# lla = lat_deg, lon_deg, alt_m
# hpr = heading_deg, pitch_deg, roll_deg
def update_state(lla_new, hpr_new, nedref_new, cam_pos_new, dlat, dlon, dalt):
    state_lock.acquire()
    global protect_ownship_lla
    global protect_cam_hpr
    global protect_nedref
    global protect_cam_pos
    global protect_lla_vel
    protect_ownship_lla = lla_new.copy()
    protect_cam_hpr = LVector3f(hpr_new)
    protect_nedref = nedref_new.copy()
    protect_cam_pos = LVector3f(cam_pos_new)
    protect_lla_vel = [dlat, dlon, dalt]
    state_lock.release()

class tile_mgr(threading.Thread):
    def __init__(self, config, dot_root):
        threading.Thread.__init__(self)
        bam_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "bam")
        self.bam_cache = tile_cache.SlippyCache(bam_dir, "", "", ext=".bam")
        self.tile_cache = { "children": {}, "protect_from_unload": False }
        self.apts_inited = False
        self.apt_mgr = apt_mgr.apt_mgr(dot_root)
        self.config = config.copy()
        basepath = os.path.dirname(os.path.realpath(__file__))
        tiles_filename = os.path.join(basepath, "../data/airports/tiles_with_runways.pkl")
        print("Loading list of tiles with runways:", tiles_filename)
        if os.path.exists(tiles_filename):
            with open(tiles_filename, "rb") as f:
                self.tiles_with_runways = pickle.load(f)
        else:
            print("no tiles_with_runways files found ...")
            self.tiles_with_runways = {}

    def get_size_dist(self, own_pos_ned, node, freeze_cam):
        if node.is_empty():
            return 0, 0

        bounds = node.getBounds()
        center = bounds.getCenter()
        radius = bounds.getRadius()
        # print("loose:", center, radius)
        # bounds = node.getTightBounds()
        # center = (bounds[0] + bounds[1]) / 2
        # print("tight:", bounds, center, (center - bounds[0]).length())
        tile_pos_ned = [ center[1], center[0], -center[2] ]
        # fixme: testing with and without radius
        # dist = np.linalg.norm(np.array(own_pos_ned) - np.array(tile_pos_ned)) - radius
        dist = np.linalg.norm(np.array(own_pos_ned) - np.array(tile_pos_ned))
        # dist = np.linalg.norm(np.array(own_pos) - np.array(tile_pos_ned)) - 0.5*radius # this is fuzzy, so average 1/2 radius?
        if dist < 1:
            dist = 1

        # print("get:", own_pos, tile_pos_ned, center, radius, dist)
        converted3DPoint = LVector3f(radius, dist, 0)
        pt2d = Point2() # This will be filled in by the method, below:
        base.camLens.project(converted3DPoint, pt2d)
        pixel_size = int(pt2d.x * base.win.getXSize())

        # screen position of center of tile
        converted3DPoint = freeze_cam.getRelativePoint(render, LVector3(center[0], center[1], center[2]))
        base.camLens.project(converted3DPoint, pt2d)
        center_dist = sqrt(pt2d.x*pt2d.x + pt2d.y*pt2d.y)
        if center_dist > 1: center_dist = 1
        factor = (1-center_dist) * 0.1 + 1
        pixel_size = pixel_size * factor
        # print("dist:", dist, pt2d.x, pt2d.y)
        # print(own_pos, tile_pos, dist, bearing)
        # print( "dist:", dist, pt2d, "pixel size:", pixel_size)
        return pixel_size, dist

    def tile_has_runway(self, zoom, x, y):
        # print("tiles_with_runways:", zoom, x, y)
        if zoom in self.tiles_with_runways:
            # print("  has:", zoom)
            if x in self.tiles_with_runways[zoom]:
                # print("    has:", zoom, x)
                if y in self.tiles_with_runways[zoom][x]:
                    # print("    has:", zoeom, x, y)
                    return True
        return False

    def recurse_cache_expand(self, cache_node, own_pos_ned, freeze_cam):
        max_size = 0
        max_name = ""
        max_node = None

        if "max_zoom" in self.config:
            max_zoom = self.config["max_zoom"]
        else:
            max_zoom = default_max_zoom_level
        if "max_rwy_zoom" in self.config:
            max_rwy_zoom = self.config["max_rwy_zoom"]
        else:
            max_rwy_zoom = max_zoom

        for key in cache_node["children"].keys():
            tile = cache_node["children"][key]
            (zoom, x, y) = tile["index"]
            # print("tile cache entry name:", tile["name"], tile["index"])
            # print("tile cache entry:", key, tile)
            size = 0
            name = ""
            node = None
            if tile["children"]:
                size, name, node = self.recurse_cache_expand(tile, own_pos_ned, freeze_cam)
            elif not tile["node"].is_empty():
                lensBounds = base.camLens.makeBounds()
                bounds = tile["node"].getBounds()
                bounds.xform(tile["node"].getParent().getMat(freeze_cam))
                if not lensBounds.contains(bounds):
                    continue

                # visible tile
                est_size, dist = self.get_size_dist(own_pos_ned, tile["node"], freeze_cam)
                # print("bounds:", tile["node"].getBounds(), "pixels:", temp_size, "vis:", visible)
                if self.tile_has_runway(zoom, x, y):
                    max_zoom = max_rwy_zoom
                if zoom < max_zoom and est_size > tex_dim * 1.0:
                    size = est_size
                    name = key
                    node = tile
            if size > max_size:
                # print("size:", size, "max_size:", max_size, key, node)
                max_size = size
                max_name = name
                max_node = node
        time.sleep(0)
        return max_size, max_name, max_node

    def recurse_cache_prune(self, cache_node, own_pos_ned, freeze_cam):
        prune_list = []
        if not cache_node["protect_from_unload"]:
            cache_node["prune"] = False
            for key in cache_node["children"].keys():
                tile = cache_node["children"][key]
                # print("tile cache entry name:", tile["name"], tile["index"])
                # print("tile cache entry:", key, tile)
                if tile["protect_from_unload"]:
                    tile["prune"] = False
                elif tile["children"]:
                    tile["prune"] = False
                    _, sub_list = self.recurse_cache_prune(tile, own_pos_ned, freeze_cam)
                    prune_list.extend(sub_list)
                elif not tile["node"].is_empty():
                    est_size, dist = self.get_size_dist(own_pos_ned, tile["node"], freeze_cam)
                    bounds = tile["node"].getBounds()
                    lensBounds = base.camLens.makeBounds()
                    bounds.xform(tile["node"].getParent().getMat(freeze_cam))
                    if lensBounds.contains(bounds):
                        visible = True
                    else:
                        visible = False
                    if False and not visible:
                        # artificially lower est_size when not visible to bias the
                        # removal algorithm and reduce the in-memory footprint and
                        # workload.
                        est_size /= 1.25
                    # print("bounds:", tile["node"].getBounds(), "pixels:", est_size)
                    if est_size < tex_dim * 0.4: # size of child
                        tile["prune"] = True
                    else:
                        tile["prune"] = False
                    #     print("prune bounds:", tile["node"].getBounds(), "pixels:", est_size)

        # see if all 4 children exist and need to be pruned
        prune_count = 0
        for key in cache_node["children"].keys():
            tile = cache_node["children"][key]
            if tile["prune"]:
                prune_count += 1
        if prune_count == 4:
            prune_list.append(cache_node)

        time.sleep(0)
        return False, prune_list

    def cache_draw(self, lat_deg, lon_deg, freeze_cam):
        size = 1200
        min_lat = 90
        max_lat = -90
        min_lon = 180
        max_lon = -180
        print("freeze cam set hpr:", freeze_cam.getHpr())
        for key in self.tile_cache["children"]:
            top_tile = self.tile_cache["children"][key]
            (zoom, x, y) = top_tile["index"]
            nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom)
            if nw_lat > max_lat: max_lat = nw_lat
            if nw_lon < min_lon: min_lon = nw_lon
            se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom)
            if se_lat < min_lat: min_lat = se_lat
            if se_lon > max_lon: max_lon = se_lon
        frame = np.zeros((size, size, 3), np.uint8)
        self.recurse_cache_draw(frame, freeze_cam, self.tile_cache, (min_lat, min_lon), (max_lat, max_lon))
        u1 = int(round((lon_deg - min_lon) / (max_lon - min_lon) * frame.shape[0]))
        v1 = frame.shape[1] - int(round((lat_deg - min_lat) / (max_lat - min_lat) * frame.shape[1]))
        cv2.circle(frame, (u1, v1), 3, (255,255,255), -1)
        cv2.imshow("cache visibility", frame)
        cv2.waitKey(1)

    def recurse_cache_draw(self, frame, freeze_cam, cache_node, min, max):
        for key in cache_node["children"].keys():
            try:
                tile = cache_node["children"][key]
                # print("tile cache entry name:", tile["name"], tile["index"])
                # print("tile cache entry:", key, tile)
                if tile["children"]:
                    self.recurse_cache_draw(frame, freeze_cam, tile, min, max)
                elif not tile["node"].isEmpty():
                    (zoom, x, y) = tile["index"]
                    nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom)
                    se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom)
                    lensBounds = base.camLens.makeBounds()
                    bounds = tile["node"].getBounds()
                    bounds.xform(tile["node"].getParent().getMat(freeze_cam))
                    if lensBounds.contains(bounds):
                        visible = True
                        color = (0,255,0)
                    else:
                        color = (0, 0, 255)
                    u1 = int(round((nw_lon - min[1]) / (max[1] - min[1]) * frame.shape[0]))
                    u2 = int(round((se_lon - min[1]) / (max[1] - min[1]) * frame.shape[0]))
                    v1 = frame.shape[1] - int(round((se_lat - min[0]) / (max[0] - min[0]) * frame.shape[1]))
                    v2 = frame.shape[1] - int(round((nw_lat - min[0]) / (max[0] - min[0]) * frame.shape[1]))
                    cv2.rectangle(frame, (u1, v1), (u2, v2), color, -1)
                    w = 1
                    cv2.line(frame, (u1,v1), (u2,v1), (0,0,0), w, cv2.LINE_AA)
                    cv2.line(frame, (u2,v1), (u2,v2), (0,0,0), w, cv2.LINE_AA)
                    cv2.line(frame, (u2,v2), (u1,v2), (0,0,0), w, cv2.LINE_AA)
                    cv2.line(frame, (u1,v2), (u1,v1), (0,0,0), w, cv2.LINE_AA)
            except:
                print("tile removed out from under us:", key)

    # reach in from outside (the main render loop reaches down here) and update
    # tile positions, this way all the tiles can be repositioned in sync with
    # the upstream nedref change.
    def recursive_update_pos(self, nedref, cache_node=None):
        count = 0
        if cache_node is None:
            count = self.recursive_update_pos(nedref, self.tile_cache)
        else:
            for key in cache_node["children"].keys():
                tile = cache_node["children"][key]
                if not tile["node"].isHidden():
                    # fixme: use externally provided nedref? not the locally updated one incase out of sync?
                    tile_pos = navpy.lla2ned(tile["center_lla"][0], tile["center_lla"][1], tile["center_lla"][2], nedref[0], nedref[1], nedref[2])
                    # print("tile:", tile)
                    # print("tile node:", tile["node"])
                    tile["node"].setPos(tile_pos[1], tile_pos[0], -tile_pos[2])
                    tile["node"].setHpr(0, nedref[0] - tile["center_lla"][0], tile["center_lla"][1] - nedref[1])
                    count += 1
                if tile["children"]:
                    count += self.recursive_update_pos(nedref, tile)
        return count

    def update_apt_mgr_pos(self, nedref):
        self.apt_mgr.update_airport_cache_pos(nedref)

    def request_tile_build(self, zoom, x, y, style):
        print("request sub process tile build:")
        self.build_proc.stdin.write("%d,%d,%d,%s\n" % (zoom, x, y, style))
        while True:
            result = self.build_proc.stdout.readline().strip()
            print("  builder:", result.strip())
            if result == "complete":
                break
            elif result == "error" or not len(result):
                quit()

    def load_tile(self, node, zoom, x, y, nedref):
        name = "%d/%d/%d" % (zoom, x, y)
        if name not in node["children"]:
            path = self.bam_cache.ensure_path_in_cache(zoom, x)
            meta_file = os.path.join(path, "%d" % y + ".meta")
            bam_file = os.path.join(path, "%d" % y + ".bam")

            if not os.path.exists(meta_file) or not os.path.exists(bam_file):
                # ask builder process to build tile
                self.request_tile_build(zoom, x, y, self.config["ground"])

            # load tile
            with open(meta_file, "rb") as f:
                data = pickle.load(f)
                center_lla = data["center_lla"]
            tile_node = NodePath( path )
            tile_node = loader.loadModel(Filename.fromOsSpecific(bam_file))
            tile = {
                "name": "%d/%d/%d" % (zoom, x, y),
                "index": (zoom, x, y),
                "node": tile_node,
                "center_lla": center_lla,
                "children": {},
                "protect_from_unload": True
            }
            tile_pos = navpy.lla2ned(tile["center_lla"][0], tile["center_lla"][1], tile["center_lla"][2], nedref[0], nedref[1], nedref[2])
            tile["node"].setPos(tile_pos[1], tile_pos[0], -tile_pos[2])
            tile["index"] = (zoom, x, y)
            cache_lock.acquire()
            node["children"][name] = tile
            cache_lock.release()
            return tile
        else:
            return None

    def run(self):
        build_cmd = ["python", "-m", "nstSimulator.world.tile_builder"]
        self.build_proc = subprocess.Popen(build_cmd, bufsize=1, text=True, stdout=subprocess.PIPE, stdin=subprocess.PIPE)

        lat_deg = None
        lon_deg = None
        alt_m = None
        cam_pos = None
        cam_hpr = None
        own_pos_ned = None
        dlat = 0
        dlon = 0
        dalt = 0
        while True:
            state_lock.acquire()
            if protect_ownship_lla is not None:
                lat_deg = protect_ownship_lla[0]
                lon_deg = protect_ownship_lla[1]
                alt_m = protect_ownship_lla[2]
                nedref = protect_nedref.copy()
                [dlat, dlon, dalt] = protect_lla_vel
                cam_pos = LVector3f(protect_cam_pos)
                cam_hpr = LVector3f(protect_cam_hpr)
            state_lock.release()

            if lat_deg is None or lon_deg is None:
                print("Waiting for position...")
                time.sleep(1)
                continue

            project_secs = 5
            own_pos_ned = navpy.lla2ned(lat_deg + dlat*project_secs, lon_deg + dlon*project_secs, alt_m + dalt*project_secs, nedref[0], nedref[1], nedref[2])

            new_nodes = []
            hide_tile = None
            prune_list = []

            if not self.apts_inited:
                apt_nodes = self.apt_mgr.init_airports(lat_deg, lon_deg, vis_range_m)
                new_nodes.extend(apt_nodes)
                self.apts_inited = True

            if "ground" in self.config:
                # check if any top level terrain tiles need to be loaded
                top_tiles = slippy_tiles.get_tiles_in_range(lat_deg, lon_deg, terrain_min_zoom_level, vis_range_m)
                for (zoom, x, y) in top_tiles:
                    name = "%d/%d/%d" % (zoom, x, y)
                    if "name" not in self.tile_cache["children"]:
                        tile = self.load_tile(self.tile_cache, zoom, x, y, nedref)
                        if tile is not None:
                            new_nodes.append(tile)

                # check if any tiles are getting too big on screen and need to be subdivided
                freeze_cam = copy.deepcopy(base.cam)
                freeze_cam.setPos(cam_pos)
                freeze_cam.setHpr(cam_hpr)
                max_size, max_name, max_node = self.recurse_cache_expand(self.tile_cache, own_pos_ned, freeze_cam)
                print("result:", max_size, max_name, max_node)
                if max_size == 0:
                    time.sleep(5)
                if max_name != "":
                    (zoom, x, y) = max_node["index"]
                    max_node["children"] = {}
                    max_node["protect_from_unload"] = True
                    tile = self.load_tile(max_node, zoom+1, 2*x, 2*y, nedref)
                    new_nodes.append(tile)
                    tile = self.load_tile(max_node, zoom+1, 2*x, 2*y+1, nedref)
                    new_nodes.append(tile)
                    tile = self.load_tile(max_node, zoom+1, 2*x+1, 2*y, nedref)
                    new_nodes.append(tile)
                    tile = self.load_tile(max_node, zoom+1, 2*x+1, 2*y+1, nedref)
                    new_nodes.append(tile)

                    # remove_queue.append(max_node)
                    hide_tile = max_node

                # todo/fixme: make sure no expansion nodes end up in the prune
                # list ... there is an edge case lurking somewhere where
                # something like this is happening.

                if True:
                    # check if any tiles need to be pruned
                    prune_flag, prune_list = self.recurse_cache_prune(self.tile_cache, own_pos_ned, freeze_cam)
                    print("prune_list:", prune_list)

            queue_lock.acquire()
            reparent_queue.extend(new_nodes)
            if hide_tile is not None:
                hide_queue.append(hide_tile)
            prune_queue.extend(prune_list)
            queue_lock.release()

            self.cache_draw(lat_deg, lon_deg, freeze_cam)
            print("tile mgr pausing...")
            time.sleep(0.1)

