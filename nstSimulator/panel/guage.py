from math import cos, sin
import numpy as np
import os
from panda3d.core import *

from PropertyTree import PropertyNode
from nstSimulator.utils.constants import d2r, m2ft

config = [
    { "texture": "data/textures/flaps_face.png" },
    { "texture": "data/textures/flaps_needle.png", "xscale": 0.8, "yscale": 0.5, "xpos": -0.8, "ypos": 0.0, "rotate_deg": { "prop": "/fcs/flaps_norm", "scale": 40 } }
]

# fixme, is there a better place/way for this?  We just want to load the font
# once, but we need panda3d initialized first.
B612_font = None
def get_B612_font():
    global B612_font
    if not B612_font:
        file_path = os.path.dirname(os.path.realpath(__file__))
        base_path = os.path.join(file_path, "../data/fonts")
        print("loading: ", os.path.join(base_path, "B612-Regular.ttf"))
        B612_font = loader.loadFont(str(Filename.fromOsSpecific(os.path.join(base_path, "B612-Regular.ttf"))))
        B612_font.setPageSize(512,512)
        B612_font.setPixelsPerUnit(80)
    return B612_font

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


def gen_ArcLine2d(color4=(0.9, 0.1, 0.1, 0.7), center_x=0, center_y=0, scale=1, width=1, start=0, end=360, steps=0, tic_list=[], label_list=[]):
    group_node = NodePath("group")
    group_node.setLightOff(1)

    if not steps:
        steps = int(round((end - start) / 10)) + 1
    if steps < 2:
        steps = 2

    divs_rad = (90 - np.linspace(start, end, steps)) * d2r
    print("divs:", divs_rad)

    ls = LineSegs()
    ls.setThickness(width)
    ls.setColor(color4)

    first = True
    for a in divs_rad:
        x1 = center_x + cos(a) * scale
        y1 = center_y + sin(a) * scale
        print(x1, y1)
        if first:
            first = False
            ls.moveTo(x1, y1, 0)
        else:
            ls.drawTo(x1, y1, 0)

    for tics in tic_list:
        for a_deg in range(start, end+1, tics[0]):
            a_rad = (90 - a_deg) * d2r
            x1 = center_x + cos(a_rad) * scale
            y1 = center_y + sin(a_rad) * scale
            x2 = center_x + cos(a_rad) * (scale*tics[1])
            y2 = center_y + sin(a_rad) * (scale*tics[1])
            ls.moveTo(x1, y1, 0)
            ls.drawTo(x2, y2, 0)

    line_node = NodePath(ls.create())
    line_node.setAntialias(AntialiasAttrib.MLine)
    line_node.reparentTo(group_node)

    for label in label_list:
        text = TextNode("text")
        text.setFont(get_B612_font())
        text.setText(label[1])
        text.setAlign(TextNode.ACenter)
        text.setTextColor(color4)
        text_node = NodePath("text")
        text_node.attachNewNode(text)
        # text_node.setScale(int(round(scale/10)))
        # text_node.setSx(int(round(scale/10)))
        # text_node.setSy(3*int(round(scale/10)))
        text_node.setScale(int(round(scale/10)), int(round(scale/10)), 3*int(round(scale/10)))
        # self.node.setBin("fixed", 1)

        bounds = text_node.getTightBounds()
        print("bounds:", bounds)
        xsize = bounds[1][0] - bounds[0][0]
        ysize = bounds[1][2] - bounds[0][2]
        # # print("ysize:", ysize)
        a_rad = (90 - label[0]) * d2r
        x1 = center_x + cos(a_rad) * scale * 1.01
        y1 = center_y + sin(a_rad) * scale * 1.01
        text_node.setPos(x1, y1, 0)
        text_node.setHpr(-label[0], -90, 0)
        text_node.reparentTo(group_node)

    return group_node

class ArcPoly2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), center_x=0, center_y=0, radius=0.5, width=0.1, start=0, end=360, steps=0):
        if not steps:
            steps = int(round((end - start) / 10)) + 1
        if steps < 2:
            steps = 2
        self.steps = steps

        divs_rad = (90 - np.linspace(start, end, self.steps)) * d2r
        print("divs:", divs_rad)

        format = GeomVertexFormat.getV3()
        self.vdata = GeomVertexData("arc", format, Geom.UHStatic)
        self.vdata.setNumRows(self.steps * 2)
        vertex = GeomVertexRewriter(self.vdata, "vertex")

        for a in divs_rad:
            x1 = cos(a) * (radius - width)
            x2 = cos(a) * (radius + width)
            y1 = sin(a) * (radius - width)
            y2 = sin(a) * (radius + width)
            print(x1, x2, y1, y2)
            vertex.addData3(center_x + x1, 0, center_y + y1)
            vertex.addData3(center_x + x2, 0, center_y + y2)

        prim = GeomTristrips(Geom.UHStatic) # don't expect geometry to change
        prim.add_consecutive_vertices(0, self.steps*2)
        prim.closePrimitive()
        geom = Geom(self.vdata)
        geom.addPrimitive(prim)
        node = GeomNode("geom")
        node.addGeom(geom)
        self.arc = aspect2d.attachNewNode(node)
        self.arc.setTwoSided(True)
        self.arc.setColor(color4)
        self.arc.setTransparency(TransparencyAttrib.MAlpha)
        #self.update(0, 0)

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
        circle = gen_ArcLine2d(color4=(1, 1, 1, 1), center_x=0, center_y=0, scale=2500, width=0.5, steps=72,
                               tic_list=[[10, 0.9], [1, 0.95]],
                               label_list=[[0, "North"], [90, "East"], [180, "South"], [270, "West"]])
        circle.setPos(0, 0, 100)
        circle.reparentTo(node)

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