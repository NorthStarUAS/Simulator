import numpy as np
from panda3d.core import *

from math import cos, sin

from nstSimulator.graphics.arcline import gen_arc_list
from nstSimulator.graphics.fonts import get_B612_font
from nstSimulator.graphics.text import Text3d
from nstSimulator.sim.lib.props import att_node, pos_node
from nstSimulator.utils.constants import m2ft

class AltitudeTapeYukikaze():
    def __init__(self):
        r1 = 400
        r2 = 15
        line_width = 2
        text_scale = 20
        green = (0, 1, 0, 1)

        self.node = render.attachNewNode("SpeedTape Yukikaze")

        self.alt_text = Text3d(color=green, val=12345, format="%.0f", scale=25, align=TextNode.ALeft, pad=2)
        self.alt_text.node.setP(1) # vertical align text a bit better
        self.alt_text.node.setPos(self.alt_text.node, r2, -400, 0)
        self.alt_text.node.reparentTo(self.node)
        self.alt_text.node.setDepthWrite(False)
        self.alt_text.node.setDepthTest(False)
        self.alt_text.node.setBin("fixed", 0)
        self.alt_text.node.setLightOff(1)

        self.speedtape = NodePath("tape")
        self.speedtape.reparentTo(self.node)

        alts100_list = np.linspace(0, 270, 28)
        labels_list = alts100_list-1
        alts1_list = np.linspace(0, 270, 271)
        arc1 = gen_arc_list(radius=r1, angle_list=alts100_list)
        arc2 = gen_arc_list(radius=r1, angle_list=labels_list)
        arc3 = gen_arc_list(radius=r1, angle_list=alts1_list)

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
            ls.moveTo(-r2, -y, x)
            ls.drawTo( r2*0.5, -y, x)

        line_node = NodePath(ls.create())
        line_node.setAntialias(AntialiasAttrib.MLine)
        line_node.reparentTo(self.speedtape)

        labels = labels_list
        for i in range(len(labels)):
            label = "%.0f" % (alts100_list[i]*100)
            if True or label != "0":
                # left side
                alt_text = TextNode("text")
                alt_text.setFont(get_B612_font())
                alt_text.setText(label)
                alt_text.setAlign(TextNode.ALeft)
                alt_text.setTextColor(green)
                text_node = NodePath("text")
                text_node.attachNewNode(alt_text)
                text_node.setScale(text_scale)
                (x, y) = arc2[i]
                text_node.setPos(r2*1.1, -y, x)
                text_node.setHpr(0, -labels[i], 0)
                text_node.reparentTo(self.speedtape)

        self.speedtape.setDepthWrite(False)
        self.speedtape.setDepthTest(False)
        self.speedtape.setBin("fixed", -1)
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

        # self.node.setDepthWrite(False, 1)  # and force this state to propagate down to children
        # self.node.setDepthTest(False, 1)
        # self.node.setBin("fixed", 0)
        # self.node.setLightOff(1)

    def update(self, nedpos):
        self.node.setPos(nedpos[1], nedpos[0], -nedpos[2])
        self.node.setHpr(-att_node.getDouble("psi_deg"), att_node.getDouble("theta_deg"), att_node.getDouble("phi_deg"))

        self.node.setY(self.node, 1600)
        self.node.setX(self.node, 400)

        alt_ft = pos_node.getDouble("geod_alt_m") * m2ft
        alt_disp = pos_node.getDouble("alt_msl_filt")
        self.speedtape.setP(self.node, (alt_ft/100))
        self.alt_text.update(alt_disp)

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
