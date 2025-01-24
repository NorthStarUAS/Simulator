#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import os
import pathlib
import pickle
import scipy
import sys
import time

from panda3d.core import *
from direct.stdpy import threading

import navpy

from . import slippy_tiles
from . import srtm2
from . import tile_cache

loadPrcFileData("", "compressed-textures 1") # compress textures when we load/save them

tile_steps_sat = 48
tile_steps_wireframe = 4

sat_mat = Material()
sat_mat.setAmbient((1, 1, 1, 1.0))
sat_mat.setDiffuse((1, 1, 1, 1.0))

poly_mat = Material()
poly_mat.setAmbient((0.3, 0.8, 0.3, 0.8))
poly_mat.setDiffuse((0.3, 0.8, 0.3, 0.8))

terra_format = GeomVertexFormat.get_v3n3t2()

class Builder():
    def __init__(self, dot_root, style):
        if style.startswith("maptiler"):
            maptiler_keyfile = os.path.join(pathlib.Path.home(), dot_root, "maptiler.txt")
            if os.path.exists(maptiler_keyfile):
                with open(maptiler_keyfile, "r") as f:
                    line = f.readline()
                    if line.startswith("key="):
                        self.maptiler_option = line.strip()
                    else:
                        self.maptiler_option = "key=" + line.strip()
                print("found maptiler key:", self.maptiler_option)
                # https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key=abcdefghijklmnopqrst
                maptiler_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "maptiler")
                self.cache = tile_cache.SlippyCache(maptiler_dir, "https://api.maptiler.com", "/tiles/satellite-v2", ext=".jpg", options=self.maptiler_option)
            else:
                print("no maptiler key found:", maptiler_keyfile)
        elif style.startswith("bing"):
            bing_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "bing")
            self.cache = tile_cache.SlippyCache(bing_dir, "http://a0.ortho.tiles.virtualearth.net", "/tiles/a{}", ext=".jpeg", options="?g=2", index_scheme="quadkey")
        elif style.startswith("google"):
            google_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "google")
            self.cache = tile_cache.SlippyCache(google_dir, "https://mt1.google.com", "/vt/lyrs=s&x={}&y={}&z={}", ext=".jpg", options="", index_scheme="google")

        self.srtm_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "srtm")
        pathlib.Path(self.srtm_dir).mkdir(parents=True, exist_ok=True)
        self.srtm_cache = srtm2.DEMCache(self.srtm_dir)

        bam_dir = os.path.join(pathlib.Path.home(), dot_root, "cache", "bam")
        self.bam_cache = tile_cache.SlippyCache(bam_dir, "", "", ext=".bam")

    def make_tin(self, zoom_level, x, y, steps, do_skirt):
        nw_lat, nw_lon = slippy_tiles.num2deg(x, y, zoom_level)
        se_lat, se_lon = slippy_tiles.num2deg(x+1, y+1, zoom_level)
        lon_size, lat_size, xsize_m, ysize_m, center_lat, center_lon = slippy_tiles.get_tile_size(x, y, zoom_level)
        dlon = lon_size / steps
        dlat = lat_size / steps
        lat_min = se_lat - dlat
        lon_max = se_lon + dlon
        lat_max = nw_lat + dlat
        lon_min = nw_lon - dlon
        print("make_tin:", zoom_level, x, y, nw_lat, nw_lon, se_lat, se_lon)

        coords = []
        is_skirt = []

        # over cover range by one step on each side for generating a skirt to hide
        # LOD and floating point rounding gaps between tiles

        if do_skirt:
            start = -1
            end = steps + 2
        else:
            start = 0
            end = steps + 1

        for c in range(start, end):
            for r in range(start, end):
                lon = nw_lon + dlon * c
                lat = se_lat + dlat * r
                coords.append( [lon, lat, 0] )

                # flag skirt points so later we can adjust their elevation artificially lower
                if r < 0 or r > steps or c < 0 or c > steps:
                    is_skirt.append(True)
                else:
                    is_skirt.append(False)
        coords = np.array(coords)

        # compute texture coordinates
        texcoords = []
        for p in coords:
            u = (p[0] - nw_lon) / lon_size
            v = (p[1] - se_lat) / lat_size
            texcoords.append([u,v])

        lat1, lon1, lat2, lon2 = srtm2.gen_tile_range(lat_min, lon_max, lat_max, lon_min)

        # for each srtm tile this region spans, interpolate as many elevation values
        # as we can, then copy the good values into zs.  When we finish all the
        # loaded tiles, we should have found elevations for the entire range of
        # points.
        for lat in range(lat1, lat2+1):
            for lon in range(lon1, lon2+1):
                srtm_tile = self.srtm_cache.get_tile(lat, lon)
                tilename = srtm2.make_tile_name(lat, lon)
                self.srtm_cache.make_smooth_patches(tilename) # if needed
                if srtm_tile is not None:
                    srtm_tile.full_interpolate(coords)

        for i in range(len(coords)):
            if is_skirt[i] and coords[i][2] > -9998:
                # artifically lower skirt elevations
                coords[i][2] -= xsize_m / 10

        # (not needed now): default val zero vs. /down/ elevation quick sanity check
        # for i in range(len(llas)):
        #     if llas[i][2] < -9998:
        #         print("Problem interpolating elevation for:", llas[i])
        #         llas[i][2] = 0.0 # ocean?

        fv = coords[:,2]
        sk = np.array(is_skirt)
        avg_elev_m = np.mean(fv[sk==False])
        # print(is_skirt, fv[sk==False])
        print("Tile average elev (m):", avg_elev_m)

        return coords, [center_lat, center_lon, avg_elev_m], texcoords, is_skirt

    def gen_terrain_node(self, zoom_level, x, y):
        do_skirt = True
        steps = tile_steps_sat
        # if style == "wireframe":
        #     do_skirt = False
        #     steps = tile_steps_wireframe

        coords, center_lla, texcoords, is_skirt = self.make_tin(zoom_level, x, y, steps, do_skirt)

        # compute Delaunay triangulation in lon/lat space
        tris = scipy.spatial.Delaunay(coords[:,:2])
        print("tris:", len(tris.simplices))
        if False:
            import matplotlib.pyplot as plt
            plt.triplot(points[:,0], points[:,1], tris.simplices)
            plt.plot(points[:,0], points[:,1], 'o')
            plt.show()

        # convert to xyz (via ned)
        points_xyz = []
        vals_xyz = []
        for i in range(len(coords)):
            ned = navpy.lla2ned(coords[i][1], coords[i][0], coords[i][2], center_lla[0], center_lla[1], center_lla[2])
            points_xyz.append( [ned[1], ned[0]] )
            vals_xyz.append(-ned[2])

        # compute face normals
        normals_xyz = [[] for x in range(len(points_xyz))]
        for t in tris.simplices:
            if is_skirt[t[0]] or is_skirt[t[1]] or is_skirt[t[2]]:
                continue
            p1 = np.array([points_xyz[t[0]][0], points_xyz[t[0]][1], vals_xyz[t[0]]])
            p2 = np.array([points_xyz[t[1]][0], points_xyz[t[1]][1], vals_xyz[t[1]]])
            p3 = np.array([points_xyz[t[2]][0], points_xyz[t[2]][1], vals_xyz[t[2]]])
            #print(p1, p2, p3)
            v1 = p2 - p1
            v2 = p3 - p1
            v = np.cross(v1, v2)
            n = v / np.linalg.norm(v)
            #print("normal:", n)
            normals_xyz[t[0]].append(n)
            normals_xyz[t[1]].append(n)
            normals_xyz[t[2]].append(n)
        #print("normals_xyz:", normals_xyz)

        # average the connected face normals_xyz to compute each individual vertex normal
        for i in range(len(normals_xyz)):
            sum = np.zeros(3)
            count = 0
            for n in normals_xyz[i]:
                if n[2] > 0.5: # something wrong here ... trying to avoid averaging normals_xyz with skirt faces
                    sum += n
                    count += 1
            if count > 0:
                avg = sum / count
                avg = avg / np.linalg.norm(avg)
                normals_xyz[i] = avg
            else:
                normals_xyz[i] = np.array([0,0,1])

        path = self.bam_cache.ensure_path_in_cache(zoom_level, x)
        meta_file = os.path.join(path, "%d" % y + ".meta")
        bam_file = os.path.join(path, "%d" % y + ".bam")

        with open(meta_file, "wb") as f:
            data = {
                "center_lla": center_lla,
            }
            pickle.dump( data, f)

        # get the 4 one-level-down 256x256 textures to make a larger 512x512 texture
        xys = [ [2*x, 2*y], [2*x, 2*y+1], [2*x+1, 2*y], [2*x+1, 2*y+1] ]
        pnms = self.cache.get_tiles_as_pnm(zoom_level+1, xys)
        pnm = PNMImage(512, 512, 3)
        pnm.copySubImage(pnms[0], 0, 0, 0, 0, 256, 256)
        pnm.copySubImage(pnms[1], 0, 256, 0, 0, 256, 256)
        pnm.copySubImage(pnms[2], 256, 0, 0, 0, 256, 256)
        pnm.copySubImage(pnms[3], 256, 256, 0, 0, 256, 256)
        surface_tex = Texture()
        surface_tex.load(pnm)
        # surface_tex = self.cache.get_tile_as_tex(zoom_level, x, y)
        surface_tex.setWrapU(Texture.WM_clamp)
        surface_tex.setWrapV(Texture.WM_clamp)
        surface_tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
        surface_tex.setAnisotropicDegree(16)
        surface_tex.generateRamMipmapImages()

        vdata = GeomVertexData("terrain", terra_format, Geom.UHStatic)
        vdata.setNumRows(len(points_xyz))

        vertex = GeomVertexWriter(vdata, "vertex")
        for i in range(len(points_xyz)):
            vertex.addData3f(points_xyz[i][0], points_xyz[i][1], vals_xyz[i])

        norm_writer = GeomVertexWriter(vdata, "normal")
        for i in range(len(normals_xyz)):
            norm_writer.addData3f(normals_xyz[i][0], normals_xyz[i][1], normals_xyz[i][2])

        tex_writer = GeomVertexWriter(vdata, 'texcoord')
        for i in range(len(texcoords)):
            tex_writer.addData2f(texcoords[i][0], texcoords[i][1])

        prim = GeomTriangles(Geom.UHStatic) # don't expect geometry to change
        for t in tris.simplices:
            prim.addVertices(t[0], t[1], t[2])
        prim.closePrimitive()

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode("geom")
        node.addGeom(geom)

        tile_node = NodePath( "%d/%d/%d" % (zoom_level, x, y) )
        tile_node.attachNewNode(node)

        # tile_node.setTwoSided(True)
        # tile_node.setAttrib(ShadeModelAttrib.make(ShadeModelAttrib.MFlat))

        # didn't seem to do anything? tile_node.setAttrib(LightRampAttrib.makeHdr2())

        tile_node.setMaterial(sat_mat)
        tile_node.setTexture(surface_tex)
        # elif style.startswith("polygon"):
        #     tile_node.setMaterial(poly_mat)
        # if style.find("wireframe") >= 0:
        #     tile_node.setRenderModeFilledWireframe(wireframe_color=(1, 1, 1, 1))
        #     # tile_node.setTransparency(TransparencyAttrib.MAlpha)

        tile_node.writeBamFile(Filename.fromOsSpecific(bam_file))

        return { "name": "%d/%d/%d" % (zoom_level, x, y),
                "index": (zoom_level, x, y),
                "node": tile_node,
                "center_lla": center_lla,
                "children": {},
                }

    # initialize([46.842, -92.194, 0], 160000, 100, do_plot=True)

def main():
    mybuilder = None

    while True:
        try:
            line = input()
        except EOFError:
            print("tile_mgr has closed the connection, tile_builder is shutting down...")
            quit()
        # print("got:", line.split(","))
        (zoom, x, y, style) = line.split(",")
        if mybuilder is None:
            mybuilder = Builder(".nstWorld", style)
        tile = mybuilder.gen_terrain_node(int(zoom), int(x), int(y))
        print("complete")
        sys.stdout.flush()

if __name__ == "__main__":
    # tile = gen_tile.gen_terrain_node(7, 30, 40, "google")
    main()
