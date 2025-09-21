import numpy as np
from panda3d.core import *

from math import cos, sin

from nstSimulator.graphics.arcline import gen_arc_list
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.graphics.text import Text3d
from nstSimulator.sim.lib.props import att_node, pos_node

class HeadingTapeYukikaze():
    def __init__(self):
        r1 = 350
        r2 = 10
        line_width = 2
        text_scale = 20
        green = (0, 1, 0, 1)

        self.node = render.attachNewNode("SpeedTape Yukikaze")

        self.hdg_text = Text3d(color=green, val=12345, format="%.0f", scale=25, align=TextNode.ACenter, pad=2)
        self.hdg_text.node.setPos(self.hdg_text.node, 0, -r1, r2*2)
        self.hdg_text.node.reparentTo(self.node)
        self.hdg_text.node.setDepthWrite(False)
        self.hdg_text.node.setDepthTest(False)
        self.hdg_text.node.setBin("fixed", 0)
        self.hdg_text.node.setLightOff(1)

        self.headingtape = NodePath("tape")
        self.headingtape.reparentTo(self.node)

        hdg10_list = np.linspace(0, 360, 13)
        labels_list = hdg10_list
        hdg1_list = np.linspace(0, 360, 145)
        arc1 = gen_arc_list(radius=r1, angle_list=hdg10_list)
        arc2 = gen_arc_list(radius=r1, angle_list=labels_list)
        arc3 = gen_arc_list(radius=r1, angle_list=hdg1_list)

        ls = LineSegs()
        ls.setThickness(line_width)
        ls.setColor(green)

        # 10 degree markers
        for i in range(len(arc1)):
            (x, y) = arc1[i]
            ls.moveTo(x, -y, -r2)
            ls.drawTo(x, -y, r2)

        # 1 degree markers
        for (x, y) in arc3:
            ls.moveTo(x, -y, -r2)
            ls.drawTo( x, -y, r2*0.5)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.headingtape)

        labels = labels_list
        for i in range(len(labels)):
            label = "%02.0f" % (hdg10_list[i] / 10)
            if label != "00":
                # left side
                alt_text = TextNode("text")
                alt_text.setFont(get_B612_font())
                alt_text.setText(label)
                alt_text.setAlign(TextNode.ACenter)
                alt_text.setTextColor(green)
                text_node = NodePath("text")
                text_node.attachNewNode(alt_text)
                text_node.setScale(text_scale)
                (x, y) = arc2[i]
                text_node.setPos(x, -y, r2*2)
                text_node.setHpr(labels[i], 0, 0)
                text_node.reparentTo(self.headingtape)

        self.headingtape.setDepthWrite(False)
        self.headingtape.setDepthTest(False)
        self.headingtape.setBin("fixed", -1)
        self.headingtape.setLightOff(1)

        # we can give the object it's own fog, but I don't know if this is
        # helpful.
        # color = (0.5, 0.5, 0.5)
        # linfog = Fog("A linear-mode Fog node")
        # linfog.setColor(*color)
        # linfog.setLinearRange(0, 1600)
        # self.speedtape.setFog(linfog)

        node = NodePath('clip')
        self.clipPlane1 = node.attachNewNode(PlaneNode('clip1'))
        self.clipPlane1.node().setPlane(Plane(0, -1, 0, 0))
        self.headingtape.setClipPlane(self.clipPlane1)

        # self.node.setDepthWrite(False, 1)  # and force this state to propagate down to children
        # self.node.setDepthTest(False, 1)
        # self.node.setBin("fixed", 0)
        # self.node.setLightOff(1)

    def update(self, nedpos):
        self.node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))

        self.node.setY(self.node, 1600)
        self.node.setZ(self.node, 300)

        hdg = att_node.getDouble("psi_deg")
        self.headingtape.setH(self.node, -hdg)
        self.hdg_text.update(hdg)

        # self.text_node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        # self.text_node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        # self.text_node.setP(-vc)

        self.clipPlane1.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        self.clipPlane1.setY(self.clipPlane1, 1600)
        # self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)

        # self.testtext.node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        # self.testtext.node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        # self.testtext.node.setY(self.testtext.node, 400)
