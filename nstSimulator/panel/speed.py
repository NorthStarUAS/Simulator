import numpy as np
from panda3d.core import *

from math import cos, sin

from nstSimulator.graphics.arcline import gen_arc_list
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.graphics.text import Text3d
from nstSimulator.sim.lib.props import att_node, vel_node
from nstSimulator.utils.constants import d2r

class SpeedTapeYukikaze():
    def __init__(self):
        r1 = 400
        r2 = 10
        r3 = 3
        line_width = 2
        text_scale = 20
        green = (0, 1, 0, 1)

        self.node = render.attachNewNode("SpeedTape Yukikaze")

        self.speedtape = NodePath("tape")
        self.speedtape.reparentTo(self.node)

        angles10_list = np.linspace(0, 270, 28)
        labels_list = angles10_list-1
        angles1_list = np.linspace(0, 270, 271)
        arc1 = gen_arc_list(radius=r1, angle_list=angles10_list)
        arc2 = gen_arc_list(radius=r1, angle_list=labels_list)
        arc3 = gen_arc_list(radius=r1, angle_list=angles1_list)

        ls = LineSegs()
        ls.setThickness(line_width)
        ls.setColor(green)

        # 10 degree markers
        for i in range(len(arc1)):
            (x, y) = arc1[i]
            ls.moveTo(-r2, -y, x)
            ls.drawTo( r2, -y, x)

        # 1 degree markers
        for (x, y) in arc3:
            ls.moveTo(-r2*0.5, -y, x)
            ls.drawTo( r2, -y, x)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.speedtape)

        labels = labels_list
        for i in range(len(labels)):
            label = "%.0f" % angles10_list[i]
            if True or label != "0":
                # left side
                self.speed_text = TextNode("text")
                self.speed_text.setFont(get_B612_font())
                self.speed_text.setText(label)
                self.speed_text.setAlign(TextNode.ARight)
                self.speed_text.setTextColor(green)
                text_node = NodePath("text")
                text_node.attachNewNode(self.speed_text)
                text_node.setScale(text_scale)
                (x, y) = arc2[i]
                text_node.setPos(-r2*1.1, -y, x)
                text_node.setHpr(0, -labels[i], 0)
                text_node.reparentTo(self.speedtape)

        self.speed_text = TextNode("text")
        self.speed_text.setFont(get_B612_font())
        self.speed_text.setText(label)
        self.speed_text.setAlign(TextNode.ARight)
        self.speed_text.setTextColor(green)
        self.text_node = NodePath("text")
        self.text_node.attachNewNode(self.speed_text)
        self.text_node.setScale(text_scale*1.2)
        (x, y) = arc2[0]
        self.text_node.setPos(0, -y, x)
        # text_node.setHpr(0, 0, 0)
        self.text_node.reparentTo(self.speedtape)

        self.speedtape.setDepthWrite(False)
        self.speedtape.setDepthTest(False)
        # self.pitch.setBin("background", 0)
        self.speedtape.setLightOff(1)

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
        self.speedtape.setClipPlane(self.clipPlane1)

        self.testtext = Text3d(val=12345, scale=20)
        self.testtext.node.reparentTo(render)
        # render.attachNewNode(self.testtext.node)

    def update(self, nedpos):
        self.speedtape.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.speedtape.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))

        self.speedtape.setY(self.speedtape, 1600)
        self.speedtape.setX(self.speedtape, -400)

        vc = vel_node.getDouble("vc_kts_filt")
        self.speedtape.setP(self.speedtape, vc)
        self.speed_text.setText("%.0f" % vc)

        # self.text_node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        # self.text_node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        self.text_node.setP(-vc)

        self.clipPlane1.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        self.clipPlane1.setY(self.clipPlane1, 1600)
        # self.clipPlane1.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), 0)

        self.testtext.node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.testtext.node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))
        self.testtext.node.setY(self.testtext.node, 400)
