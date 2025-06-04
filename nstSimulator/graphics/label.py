from panda3d.core import *

from PropertyTree import PropertyNode

from nstSimulator.utils.graphics.fonts import get_B612_font

def parse_prop(prop):
    pos = prop.rfind("/")
    if pos >= 0:
        node = PropertyNode(prop[:pos])
        attr = prop[pos+1:]
    else:
        node = None
        attr = ""
    return node, attr

class Label2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), bg4=None, size=0.5, prop="", format="%.0f kts", scale=1.0, pad=0.1):
        if len(prop):
            self.prop_node, self.prop_attr = parse_prop(prop)
        else:
            self.prop_node = None
        self.scale = scale
        self.format = format

        self.node = aspect2d.attachNewNode("label")
        self.node.setScale(size)

        val = 123
        self.text = TextNode("text")
        self.text.setFont(get_B612_font())
        if self.prop_node:
            self.text.setText(self.format % val)
        else:
            self.text.setText(self.format)
        self.text.setAlign(TextNode.ACenter)
        self.text.setTextColor(color4)
        self.text_node = NodePath("text")
        self.text_node.setBin("fixed", 1)
        self.text_node.reparentTo(self.node)
        self.text_node.attachNewNode(self.text)
        # self.node = aspect2d.attachNewNode(self.text)

        bounds = self.text_node.getTightBounds()
        print("bounds:", bounds)
        xsize = bounds[1][0] - bounds[0][0]
        ysize = bounds[1][2] - bounds[0][2]
        # print("ysize:", ysize)
        self.text_node.setPos(0, 0, 0 - 0.43*ysize)

        if bg4 is not None:
            cm = CardMaker('background')
            cm.setFrame(-xsize*(0.5+pad), +xsize*(0.5+pad), -ysize*(0.5+pad), +ysize*(0.5+pad))
            cm.setColor(bg4)
            self.bg_node = NodePath("bg")
            self.bg_node.attachNewNode(cm.generate())
            self.bg_node.setTransparency(TransparencyAttrib.MAlpha)
            self.bg_node.setBin("fixed", 0)
            self.bg_node.reparentTo(self.node)
        else:
            self.bg_node = None

    def setDrawOrder(self, val):
        self.text_node.setBin("fixed", val)
        if self.bg_node:
            self.bg_node.setBin("fixed", val-1)

    def update(self):
        if self.prop_node:
            val = self.prop_node.getDouble(self.prop_attr) * self.scale
            self.text.setText(self.format % val)
        else:
            self.text.setText(self.format)
