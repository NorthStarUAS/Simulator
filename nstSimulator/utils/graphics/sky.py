from panda3d.core import *

from .arc import gen_arc

class SkyDome():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), radius=1, width=0.1, start_deg=0, end_deg=360, steps=10):
        outer_coords = gen_arc(radius, start_deg, end_deg, steps)
        inner_coords = gen_arc(radius-width, start_deg, end_deg, steps)

        format = GeomVertexFormat.getV3()
        self.vdata = GeomVertexData("arc", format, Geom.UHStatic)
        self.vdata.setNumRows(self.steps * 2)
        vertex = GeomVertexRewriter(self.vdata, "vertex")

        for i in range(len(outer_coords)):
            (x1, y1) = outer_coords[i]
            (x2, y2) = inner_coords[i]
            vertex.addData3(x1, 0, y1)
            vertex.addData3(x2, 0, y2)

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
