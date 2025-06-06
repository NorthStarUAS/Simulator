from math import cos, sin
import numpy as np
from panda3d.core import *

from PropertyTree import PropertyNode
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.graphics.arcline import ArcLine2d
from nstSimulator.graphics.circlepoly import CirclePoly2d
from nstSimulator.graphics.label import Label2d
from nstSimulator.utils.constants import d2r

config = [
    { "texture": "data/textures/flaps_face.png" },
    { "texture": "data/textures/flaps_needle.png", "xscale": 0.8, "yscale": 0.5, "xpos": -0.8, "ypos": 0.0, "rotate_deg": { "prop": "/fcs/flaps_norm", "scale": 40 } }
]

def parse_prop(prop):
    pos = prop.rfind("/")
    if pos >= 0:
        node = PropertyNode(prop[:pos])
        attr = prop[pos+1:]
    else:
        node = None
        attr = ""
    return node, attr

class Animation():
    def __init__(self):
        pass

class Text2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), center_x=0, center_y=0, size=0.5, prop="/constants/zero", format="%.0f kts", scale=1.0, pad=0.1):
        self.prop_node, self.prop_attr = parse_prop(prop)
        self.scale = scale
        self.format = format

        val = 987
        self.text = TextNode("text")
        self.text.setFont(get_B612_font())
        self.text.setText(self.format % val)
        self.text.setAlign(TextNode.ACenter)
        self.text.setTextColor(color4)
        self.node = aspect2d.attachNewNode(self.text)
        self.node.setScale(size)
        self.node.setBin("fixed", 1)

        bounds = self.node.getTightBounds()
        print("bounds:", bounds)
        xsize = bounds[1][0] - bounds[0][0]
        ysize = bounds[1][2] - bounds[0][2]
        # print("ysize:", ysize)
        self.node.setPos(center_x, 0, center_y - 0.43*ysize)

        cm = CardMaker('background')
        cm.setFrame(center_x-xsize*(0.5+pad), center_x+xsize*(0.5+pad), center_y-ysize*(0.5+pad), center_y+ysize*(0.5+pad))
        cm.setColor(0, 0, 0, 0.2)
        self.background = aspect2d.attachNewNode(cm.generate())
        self.background.setTransparency(TransparencyAttrib.MAlpha)
        self.background.setBin("fixed", 0)

    def update(self):
        val = self.prop_node.getDouble(self.prop_attr) * self.scale
        self.text.setText(self.format % val)

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

        # ArcPoly2d((0.9, 0.2, 0.2, 0.5), 0, 0, 0.5, 0.2, 10, 350)
        self.asi = Text2d((0.2, 0.9, 0.2, 0.75), -0.75, 0, 0.1, "/velocity/vc_kts_filt", format="%.0f kt")
        self.alt = Text2d((0.2, 0.9, 0.2, 0.75),  0.75, 0, 0.1, "/position/alt_msl_filt", format="%.0f msl")

        node = render.attachNewNode("circle")
        circle = ArcLine2d(color4=(1, 1, 1, 1), radius=2500, width=1, steps=72,
                           tic_list=[[10, 0.85], [1, 0.95]],
                           label_list=[[0, "North", 1.03], [90, "East", 1.03], [180, "South", 1.03], [270, "West", 1.03]])
        circle.group_node.setPos(0, 0, 100)
        circle.group_node.reparentTo(node)

        circle = CirclePoly2d(color4=(0.2, 0.2, 0.9, 0.5), radius=0.1, start_deg=0, end_deg=360, steps=36)
        circle.node.setBin("fixed", 0)

        label = Label2d(color4=(0.2, 0.9, 0.2, 0.75), size=0.1, format="ABC")
        label.setDrawOrder(2)

        # for i, layer in enumerate(config):
        #     if "texture" in layer: texture = layer["texture"]
        #     else: texture = ""
        #     if "xscale" in layer: xscale = layer["xscale"]
        #     else: xscale = 1
        #     if "yscale" in layer: yscale = layer["yscale"]
        #     else: yscale = 1
        #     handle = Layer(texture, size, xscale, yscale)
        #     handle.update(x, y)
        #     layer["handle"] = handle
        #     self.layers.append(layer)

    def update(self):
        self.asi.update()
        self.alt.update()

        for i, layer in enumerate(self.layers):
            handle = layer["handle"]
            if "rotate_deg" in layer:
                anim = layer["rotate_deg"]
                val = anim["prop"]
                scale = anim["scale"]
                handle.setHpr(0, 0, val*scale)

        # self.face.setPos(self.x*base.getAspectRatio(), 0, self.y + 0.5*self.size)
        # self.needle.setPos((self.x*base.getAspectRatio() - self.size*0.8), 0, self.y + self.size)
        # import time
        # from math import sin
        # pos = sin(time.time()) * 0.5 + 0.5
        # self.needle.setHpr(0, 0, pos*40)