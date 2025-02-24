from math import cos, sin
from panda3d.core import *

import sys
sys.path.append("..")
from nstSimulator.utils.constants import d2r

config = [
    { "texture": "data/textures/flaps_face.png" },
    { "texture": "data/textures/flaps_needle.png", "xscale": 0.8, "yscale": 0.5, "xpos": -0.8, "ypos": 0.0, "rotate_deg": { "prop": "/fcs/flaps_norm", "scale": 40 } }
]

class Animation():
    def __init__(self):
        pass

class Layer():
    def __init__(self, tex_path, size, xscale, yscale):
        self.size = size
        self.xscale = xscale
        self.yscale = yscale

        cm = CardMaker("guage layer")
        cm.setFrame(-size, size, -size, size)
        self.handle = aspect2d.attachNewNode(cm.generate())
        self.handle.setTransparency(TransparencyAttrib.MAlpha)
        if len(tex_path):
            tex = loader.loadTexture(tex_path)
            self.handle.setTexture(tex)
        self.handle.setPos(x*base.getAspectRatio() - size*xscale, 0, y+size*yscale)

    def update(self, x, y):
        self.handle.setPos(x*base.getAspectRatio(), 0, y + 0.5*self.size)

class Guage():
    def __init__(self, size, x, y):
        self.x = x
        self.y = y
        self.size = size
        self.layers = []

        for i, layer in enumerate(config):
            if "texture" in layer: texture = layer["texture"]
            else: texture = ""
            if "xscale" in layer: xscale = layer["xscale"]
            else: xscale = 1
            if "yscale" in layer: yscale = layer["yscale"]
            else: yscale = 1
            handle = Layer(texture, size, xscale, yscale)
            handle.update(x, y)
            layer["handle"] = handle
            self.layers.append(layer)

    def update(self, pos):
        for i, layer in enumerate(self.layers):
            handle = layer["handle"]
            if "rotate_deg" in layer:
                anim = layer["rotate_deg"]
                val = anim["prop"]
                scale = anim["scale"]
                handle.setHpr(0, 0, val*scale)

        self.face.setPos(self.x*base.getAspectRatio(), 0, self.y + 0.5*self.size)
        self.needle.setPos((self.x*base.getAspectRatio() - self.size*0.8), 0, self.y + self.size)
        # import time
        # from math import sin
        # pos = sin(time.time()) * 0.5 + 0.5
        self.needle.setHpr(0, 0, pos*40)