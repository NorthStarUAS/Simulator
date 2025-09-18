import numpy as np
from panda3d.core import *

from nstSimulator.graphics.arcline import gen_arc_list
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.sim.lib.props import att_node

class PitchLadderF16():
    def __init__(self):
        r1 = 400
        r2 = 60
        r3 = 10
        line_width = 2
        text_scale = 8
        green = (0, 1, 0, 1)

        self.group_node = NodePath("group")
        self.group_node.setLightOff(1)
        self.pitch = render.attachNewNode("f16")

        angles_list = np.linspace(-80, 80, 17)
        edge_angles_list = angles_list * 1.02
        lip_angles_list = angles_list - 0.4 * angles_list / np.abs(angles_list)
        arc1 = gen_arc_list(radius=r1, angle_list=angles_list)
        arc2 = gen_arc_list(radius=r1, angle_list=edge_angles_list)
        arc3 = gen_arc_list(radius=r1, angle_list=lip_angles_list)

        angles5_list = np.linspace(-85, 85, 18)
        print("angles5:", angles5_list)
        edge_angles5_list = angles5_list * 1.008
        arc4 = gen_arc_list(radius=r1, angle_list=angles5_list)
        arc5 = gen_arc_list(radius=r1, angle_list=edge_angles5_list)

        ls = LineSegs()
        ls.setThickness(line_width)
        ls.setColor(green)

        # 10 degree markers
        for i in range(len(arc1)):
            (x1, y1) = arc2[i]
            (x2, y2) = arc1[i]
            (x3, y3) = arc3[i]
            ls.moveTo(-r2, y1, x1)
            if angles_list[i] >= 0:
                ls.drawTo(-r3, y2, x2)
            else:
                # make dashed
                dr = r2-r3
                dy = y2-y1
                dx = x2-x1
                ls.drawTo(-r2+dr*0.25, y1+dy*0.25, x1+dx*0.25)
                ls.moveTo(-r2+dr*0.375, y1+dy*0.375, x1+dx*0.375)
                ls.drawTo(-r2+dr*0.625, y1+dy*0.625, x1+dx*0.625)
                ls.moveTo(-r2+dr*0.75, y1+dy*0.75, x1+dx*0.75)
                ls.drawTo(-r2+dr*1.0, y1+dy*1.0, x1+dx*1.0)
            ls.drawTo(-r3, y3, x3)
            ls.moveTo(r2, y1, x1)
            if angles_list[i] >= 0:
                ls.drawTo(r3, y2, x2)
            else:
                # make dashed
                dr = r2-r3
                dy = y2-y1
                dx = x2-x1
                ls.drawTo(r2-dr*0.25, y1+dy*0.25, x1+dx*0.25)
                ls.moveTo(r2-dr*0.375, y1+dy*0.375, x1+dx*0.375)
                ls.drawTo(r2-dr*0.625, y1+dy*0.625, x1+dx*0.625)
                ls.moveTo(r2-dr*0.75, y1+dy*0.75, x1+dx*0.75)
                ls.drawTo(r2-dr*1.0, y1+dy*1.0, x1+dx*1.0)
            ls.drawTo(r3, y3, x3)

        # 5 degree markers
        for i in range(len(arc4)):
            (x1, y1) = arc5[i]
            (x2, y2) = arc4[i]
            ls.moveTo(-r2*0.5, y1, x1)
            ls.drawTo(-r3, y2, x2)
            ls.moveTo(r2*0.5, y1, x1)
            ls.drawTo(r3, y2, x2)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.pitch)

        labels = angles_list
        for i in range(len(labels)):
            label = "%.0f" % labels[i]
            if label != "0":
                # left side
                text = TextNode("text")
                text.setFont(get_B612_font())
                text.setText(label)
                text.setAlign(TextNode.ARight)
                text.setTextColor(green)
                text_node = NodePath("text")
                text_node.attachNewNode(text)
                text_node.setScale(text_scale)
                (x, y) = arc2[i]
                text_node.setPos(-r2*1.05, y, x-2)
                text_node.setHpr(0, labels[i], 0)
                text_node.reparentTo(self.pitch)

                # repeat for right side of line
                text = TextNode("text")
                text.setFont(get_B612_font())
                text.setText(label)
                text.setAlign(TextNode.ALeft)
                text.setTextColor(green)
                text_node = NodePath("text")
                text_node.attachNewNode(text)
                text_node.setScale(text_scale)
                (x, y) = arc2[i]
                text_node.setPos(r2*1.05, y, x-2)
                text_node.setHpr(0, labels[i], 0)
                text_node.reparentTo(self.pitch)

        # node = NodePath('clip')
        # self.clipPlane1 = node.attachNewNode(PlaneNode('clip1'))
        # self.clipPlane1.node().setPlane(Plane(0, 0, -1, 0))
        # self.clipPlane2 = node.attachNewNode(PlaneNode('clip2'))
        # self.clipPlane2.node().setPlane(Plane(0, 0, 1, 0))
        # self.pitch.setClipPlane(self.clipPlane1)
        # self.pitch.setClipPlane(self.clipPlane2)

        self.pitch.setDepthWrite(False)
        self.pitch.setDepthTest(False)
        # self.pitch.setBin("background", 0)
        self.pitch.setLightOff(1)

    def update(self, nedpos):
        self.pitch.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.pitch.setHpr(-att_node.getDouble("psi_deg"), 0, 0)
        # self.clipPlane1.setPos(nedpos[1], nedpos[0], -nedpos[2]+115)
        # self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)
        # self.clipPlane2.setPos(nedpos[1], nedpos[0], -nedpos[2]-100)
        # self.clipPlane2.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)

# p = PitchLadder()