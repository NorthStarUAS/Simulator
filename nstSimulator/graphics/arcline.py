# build a circle (arc) out of a line primative, optionally add tics and labels.

from panda3d.core import *

from nstSimulator.utils.graphics.fonts import get_B612_font
from .arc import gen_arc, gen_arc_list

class ArcLine2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), radius=1, width=1, start_deg=0, end_deg=360, steps=10, font_scale=(0.1, 0.3), tic_list=[], label_radius=1, label_list=[]):
        self.group_node = NodePath("group")
        self.group_node.setLightOff(1)

        ring_coords = gen_arc(radius, start_deg, end_deg, steps)

        ls = LineSegs()
        ls.setThickness(width)
        ls.setColor(color4)

        first = True
        for (x, y) in ring_coords:
            if first:
                first = False
                ls.moveTo(x, y, 0)
            else:
                ls.drawTo(x, y, 0)

        for [tic_steps, tic_scale] in tic_list:
            tic_outer = gen_arc(radius, start_deg, end_deg, tic_steps)
            tic_inner = gen_arc(radius*tic_scale, start_deg, end_deg, tic_steps)

            for i in range(len(tic_outer)):
                (x1, y1) = tic_outer[i]
                (x2, y2) = tic_inner[i]
                ls.moveTo(x1, y1, 0)
                ls.drawTo(x2, y2, 0)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.group_node)

        angle_list = []
        for label in label_list:
            angle_list.append(label[0])
        label_coords = gen_arc_list(label_radius, angle_list)

        for i in range(len(label_list)):
            label = label_list[i]
            text = TextNode("text")
            text.setFont(get_B612_font())
            text.setText(label[1])
            text.setAlign(TextNode.ACenter)
            text.setTextColor(color4)
            text_node = NodePath("text")
            text_node.attachNewNode(text)
            text_node.setScale(font_scale[0], font_scale[0], font_scale[1])
            (x, y) = label_coords[i]
            text_node.setPos(x, y, 0)
            text_node.setHpr(-label[0], -90, 0)
            text_node.reparentTo(self.group_node)
