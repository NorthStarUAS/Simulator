from panda3d.core import *

from .arc import gen_arc, gen_arc_list
from .arcline import ArcLine2d

class SkyDome():
    def __init__(self):
        fog = LVector4f(0.533, 0.639, 0.722, 1)
        # skyblue = LVector4f(0.529, 0.808, 0.922, 1)
        skyblue = LVector4f(0.09, 0.11, 0.71, 1)
        r1 = 500
        sky_format = GeomVertexFormat.get_v3c4()

        self.sky = render.attachNewNode("sky")

        # varc = gen_arc(r1, start_deg=0, end_deg=90, divs=9)
        vangles = [-10, 0, 2,   4,   8,    20,  40,   60,  90]
        vblend =  [  1, 1, 0.8, 0.5, 0.15, 0.1, 0.05, 0.0, 0.0]
        varc = gen_arc_list(r1, angle_list=vangles)
        print("varc:", varc)
        arc_list = []
        for x1, y1 in varc[:-1]:
            print(" ", x1, y1)
            arc = gen_arc(radius=y1, divs=36)
            arc_list.append(arc)

            # temp visual debug
            # arc = ArcLine2d(color4=(1, 1, 1, 1), radius=y1*0.95, width=1, steps=36)
            # arc.group_node.setPos(0, 0, x1)
            # arc.group_node.reparentTo(self.sky)

        print("sky dome")

        for i in range(len(arc_list)-1):
            arc1 = arc_list[i]
            arc2 = arc_list[i+1]

            vdata = GeomVertexData("sky", sky_format, Geom.UHStatic)
            vdata.setNumRows(len(arc1)*2)

            vertex = GeomVertexWriter(vdata, "vertex")

            for j in range(len(arc1)):
                vertex.addData3(arc1[j][0], arc1[j][1], varc[i][0])
                vertex.addData3(arc2[j][0], arc2[j][1], varc[i+1][0])

            blend1 = vblend[i]
            blend2 = vblend[i+1]
            color1 = fog * blend1 + skyblue * (1-blend1)
            color2 = fog * blend2 + skyblue * (1-blend2)
            color_writer = GeomVertexWriter(vdata, 'color')
            for j in range(len(arc1)):
                color_writer.addData4(color1)
                color_writer.addData4(color2)

            prim = GeomTristrips(Geom.UHStatic) # don't expect geometry to change
            prim.add_consecutive_vertices(0, len(arc1)*2)
            prim.add_consecutive_vertices(0, 2)  # loop to start
            prim.closePrimitive()

            geom = Geom(vdata)
            geom.addPrimitive(prim)

            node = GeomNode("geom")
            node.addGeom(geom)

            surface = self.sky.attachNewNode(node)
            surface.setTwoSided(True)
            # surface.setMaterial(ribbonMat)
            # surface.setTexture(tex)
            # surface.setTransparency(TransparencyAttrib.MAlpha)

        # self.sky.setTwoSided(True)
        # self.sky.setMaterial(compassMat)
        # self.sky.setTexture(tex)
        # self.sky.setTransparency(TransparencyAttrib.MAlpha)
        self.sky.setDepthWrite(False)
        self.sky.setDepthTest(False)
        # node.setDepthWrite(False)
        # node.setDepthTest(False)
        self.sky.setBin("background", 0)

    def update(self, nedpos):
        self.sky.setPos(nedpos[1], nedpos[0], -nedpos[2])
