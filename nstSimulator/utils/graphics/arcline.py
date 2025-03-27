# build a circle (arc) out of a line primative, optionally add tics and labels.

from math import cos, sin
import numpy as np
from panda3d.core import *

from nstSimulator.utils.constants import d2r
from nstSimulator.utils.graphics.fonts import get_B612_font

class ArcLine2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), scale=1, width=1, start=0, end=360, steps=0, font_scale=(0.1, 0.3), tic_list=[], label_list=[]):
        self.group_node = NodePath("group")
        self.group_node.setLightOff(1)

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
            x1 = cos(a) * scale
            y1 = sin(a) * scale
            print(x1, y1)
            if first:
                first = False
                ls.moveTo(x1, y1, 0)
            else:
                ls.drawTo(x1, y1, 0)

        for tics in tic_list:
            for a_deg in range(start, end+1, tics[0]):
                a_rad = (90 - a_deg) * d2r
                x1 = cos(a_rad) * scale
                y1 = sin(a_rad) * scale
                x2 = cos(a_rad) * (scale*tics[1])
                y2 = sin(a_rad) * (scale*tics[1])
                ls.moveTo(x1, y1, 0)
                ls.drawTo(x2, y2, 0)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.group_node)

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
            text_node.setScale(font_scale[0], font_scale[0], font_scale[1])
            # self.node.setBin("fixed", 1)

            a_rad = (90 - label[0]) * d2r
            x1 = cos(a_rad) * scale * label[2]
            y1 = sin(a_rad) * scale * label[2]
            text_node.setPos(x1, y1, 0)
            text_node.setHpr(-label[0], -90, 0)
            text_node.reparentTo(self.group_node)
