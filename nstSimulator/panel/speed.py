import numpy as np
from panda3d.core import *

from math import cos, sin

from nstSimulator.graphics.arcline import gen_arc_list
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.sim.lib.props import att_node, vel_node
from nstSimulator.utils.constants import d2r

class SpeedTapeYukikaze():
    def __init__(self):
        r1 = 400
        r2 = 5
        r3 = 3
        line_width = 2
        text_scale = 40
        green = (1, 0, 0, 1)

        self.group_node = NodePath("group")
        self.group_node.setLightOff(1)
        self.speedtape = render.attachNewNode("SpeedTape Yukikaze")

        angles10_list = np.linspace(0, 360, 37)
        angles1_list = np.linspace(0, 360, 361)
        arc1 = gen_arc_list(radius=r1, angle_list=angles10_list)
        arc2 = gen_arc_list(radius=r1, angle_list=angles1_list)

        ls = LineSegs()
        ls.setThickness(line_width)
        ls.setColor(green)

        # 10 degree markers
        for i in range(len(arc1)):
            (x, y) = arc1[i]
            # print("xy: %.1f %.1f" % (x, y))
            ls.moveTo(-r2, -y, x)
            ls.drawTo( r2, -y, x)

        # # 1 degree markers
        # for i in range(len(arc4)):
        #     (x1, y1) = arc5[i]
        #     (x2, y2) = arc4[i]
        #     ls.moveTo(-r2*0.5, y1, x1)
        #     ls.drawTo(-r3, y2, x2)
        #     ls.moveTo(r2*0.5, y1, x1)
        #     ls.drawTo(r3, y2, x2)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.speedtape)

        labels = angles10_list
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
                (x, y) = arc1[i]
                text_node.setPos(-r2*1.05, -y, x-2)
                text_node.setHpr(0, -labels[i], 0)
                text_node.reparentTo(self.speedtape)

                # # repeat for right side of line
                # text = TextNode("text")
                # text.setFont(get_B612_font())
                # text.setText(label)
                # text.setAlign(TextNode.ALeft)
                # text.setTextColor(green)
                # text_node = NodePath("text")
                # text_node.attachNewNode(text)
                # text_node.setScale(text_scale)
                # (x, y) = arc1[i]
                # text_node.setPos(r2*1.05, y, x-2)
                # text_node.setHpr(0, labels[i], 0)
                # text_node.reparentTo(self.speedtape)

        # node = NodePath('clip')
        # self.clipPlane1 = node.attachNewNode(PlaneNode('clip1'))
        # self.clipPlane1.node().setPlane(Plane(0, 0, -1, 0))
        # self.clipPlane2 = node.attachNewNode(PlaneNode('clip2'))
        # self.clipPlane2.node().setPlane(Plane(0, 0, 1, 0))
        # self.pitch.setClipPlane(self.clipPlane1)
        # self.pitch.setClipPlane(self.clipPlane2)

        self.speedtape.setDepthWrite(False)
        self.speedtape.setDepthTest(False)
        # self.pitch.setBin("background", 0)
        self.speedtape.setLightOff(1)

    def update(self, nedpos):
        self.speedtape.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.speedtape.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        # self.clipPlane1.setPos(nedpos[1], nedpos[0], -nedpos[2]+115)
        # self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)
        # self.clipPlane2.setPos(nedpos[1], nedpos[0], -nedpos[2]-100)
        # self.clipPlane2.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)

        self.speedtape.setY(self.speedtape, 1600)
        self.speedtape.setX(self.speedtape, -400)

        vc = vel_node.getDouble("vc_kts_filt")
        self.speedtape.setP(self.speedtape, vc)
        # project_forward = 400
        # angleRadians = -course_deg * d2r
        # angleRadians = -att_node.getDouble("psi_deg") * d2r
        # self.speedtape.setPos(nedpos[1] - project_forward * sin(angleRadians), nedpos[0] + project_forward * cos(angleRadians), -nedpos[2] - 15)

# p = PitchLadder()