#!/usr/bin/env python3

import argparse
import cv2  # pip install opencv-python
import importlib.metadata, importlib.util
import numpy as np

from direct.showbase.ShowBase import ShowBase # pip install panda3d
from direct.task import Task
from panda3d.core import *

from nstSimulator.graphics.display import Display
from nstSimulator.graphics import fog, light
from nstSimulator.panel import panel
from nstSimulator.world import world
try:
    if float(importlib.metadata.version("nstSimulator")) < 1.3:
        print("Please upgrade the nstSimulator package to v1.3 or higher.")
        print("Cannot continue.")
        quit()
except:
    print("nstSimulator package not installed, so using local tree...")

from lib_vis import comms_mgr

# these are supposedly the options needed for deferred texture loading...
# loadPrcFileData("", "preload-textures 0")
# loadPrcFileData("", "preload-simple-textures 1")
# loadPrcFileData("", "simple-image-size 16 16")
# loadPrcFileData("", "compressed-textures 1")
# loadPrcFileData("", "allow-incomplete-render 1")

# seems like a good idea
loadPrcFileData("", "compressed-textures 1") # compress textures when we load/save them

# antialiasing
loadPrcFileData("", "multisamples 1")

# Performance monitors
loadPrcFileData("", "show-frame-rate-meter 1") # uncomment to show FPS meter
loadPrcFileData("", "want-pstats 1") # uncomment for pstats
loadPrcFileData("", "pstats-tasks 1") # uncomment for pstats on tasks
loadPrcFileData("", "pstats-python-profiler 1") # uncomment for pstats on tasks

parser = argparse.ArgumentParser(description="Optical flow based velocity vector experiments")
args = parser.parse_args()

config = {
    "window": {
        "height": 720,
        "width": 1280,
    },
    "display": [
        { "fov_deg": 60, "offset_deg": 0, "label": "Main Display" },
    ],
    "fog": {
        "color": (0.533, 0.639, 0.722),
        # "color": (0.573, 0.569, 0.731),
    },
    "light": {
        "sun": (1, 1, 1, 1),
        "sun_vector": (0.2, 0.2, -0.8),
        "ambient": (0.4, 0.4, 0.4, 1),
    },
    "panel": {
        "enambel": "sure"
    },
    "world": {
        # "airports": True,  # future devel
        "ground": "bing",    # maptiler, bing, google, or polygon.  Add +wireframe (i.e. polygon+wireframe) to show the underlying mesh structure as wireframe
        # "max_zoom": 17,      # 15 is max for maptiler, 17 good for bing (18 works, but greatly increases the workload and the tile loading can get way behind)
        # "max_rwy_zoom": 19,
        "sky": "skybox",     # skybox
    },
}

class SceneryViewer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        w, h = 1280, 1024
        winconfig = config["window"]
        if "width" in winconfig:
            w = winconfig["width"]
        if "height" in winconfig:
            h = winconfig["height"]
        wp = WindowProperties()
        # wp.setFullscreen(True)
        wp.setSize(w, h)
        wp.setTitle("Scenery Viewer")
        base.win.requestProperties(wp)

        self.comms_mgr = comms_mgr.CommsManager()

        # display (window / camera) setup
        if "display" not in config:
            print("Error! The config is missing a 'display' section so we cannot continue.")
            quit()
        self.displayList = []
        for i, display_config in enumerate(config["display"]):
            if i == 0:
                # special case first window/camera/lens (already created by default)
                mywin = base.win
            else:
                mywin = base.openWindow()
            wp.setTitle(display_config["label"])
            mywin.requestProperties(wp)
            mycam = base.camList[-1]
            self.displayList.append( Display(mywin, mycam, display_config) )

        if "fog" not in config:
            print("Error! The config is missing a 'fog' section so we cannot continue.")
        self.fog = fog.FogMgr(config["fog"])

        if "light" not in config:
            print("Error! The config is missing a 'light' section so we cannot continue.")
        self.light = light.LightMgr(config["light"])

        if False:
            # Load the environment model.
            self.scene = self.loader.loadModel("models/environment")
            # Apply scale and position transforms on the model.
            #self.scene.setScale(0.25, 0.25, 0.25)
            #self.scene.setPos(-8, 42, 0)
            #self.scene.reparentTo(render)

        if False:
            # big coordinate system axis to show alignment
            self.zup = self.loader.loadModel("models/zup-axis")
            self.zup.setScale(10, 10, 10)
            self.zup.reparentTo(render)

        self.panel = None
        if "panel" in config:
            self.panel = panel.Panel()

        self.world = None
        if "world" in config:
            self.world = world.World(config["world"], self.comms_mgr.nedref_time)

        self.taskMgr.add(self.updateTask, "updateTask")

        if False:
            self.taskMgr.add(self.postRenderTask, "screenshot", priority=55)  # after igLoop, but before audioLoop

        render.setAntialias(AntialiasAttrib.MMultisample, 1)  # increase n for better AA quality?

    def updateTask(self, task):
        # communicate
        self.comms_mgr.update()

        ground_elev_m = self.world.get_elevation(self.comms_mgr.lla[0], self.comms_mgr.lla[1])
        for display in self.displayList:
            display.update(self.comms_mgr.nedpos, self.comms_mgr.nedvel, self.comms_mgr.hpr_deg, ground_elev_m)

        self.comms_mgr.send(ground_elev_m)

        self.fog.update(-self.comms_mgr.nedpos[2])

        if self.panel:
            self.panel.update()

        if self.world:
            self.world.update(self.displayList[0].cam_pos, self.displayList[0].cam_hpr, self.comms_mgr.nedref, self.comms_mgr.nedref_time, self.comms_mgr.lla,
                              self.comms_mgr.dlat, self.comms_mgr.dlon, self.comms_mgr.dalt)

        return Task.cont

    def postRenderTask(self, task):
        # print("screenshot")
        tex = base.camNode.getDisplayRegion(0).getScreenshot()
        if tex is not None:
            data = tex.getRamImage()
            frame = np.frombuffer(data, np.uint8)
            frame.shape = (tex.getYSize(), tex.getXSize(), tex.getNumComponents())
            frame = cv2.flip(frame, 0)
            cv2.imshow("grab", frame)
            cv2.waitKey(1)
        return Task.cont

    def myShutdown(self):
        pass

app = SceneryViewer()
app.run()
