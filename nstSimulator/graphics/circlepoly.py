from panda3d.core import *

from .arc import gen_arc

# fixme: this could be built on top of the polygon class (arc -> filled poly)
# and could even support disks with holes

class CirclePoly2d():
    def __init__(self, color4=(0.9, 0.1, 0.1, 0.7), radius=0.5, start_deg=0, end_deg=360, steps=10):
        perim_coords = gen_arc(radius, start_deg, end_deg, steps)

        format = GeomVertexFormat.getV3()
        self.vdata = GeomVertexData("circle", format, Geom.UHStatic)
        self.vdata.setNumRows(steps + 2)
        vertex = GeomVertexRewriter(self.vdata, "vertex")

        vertex.addData3(0, 0, 0)  # center point
        for i in range(len(perim_coords)):
            (x, y) = perim_coords[i]
            vertex.addData3(x, 0, y)
        (x, y) = perim_coords[0]
        vertex.addData3(x, 0, y)

        prim = GeomTrifans(Geom.UHStatic) # don't expect geometry to change
        prim.add_consecutive_vertices(0, steps + 2)
        prim.closePrimitive()
        geom = Geom(self.vdata)
        geom.addPrimitive(prim)
        node = GeomNode("geom")
        node.addGeom(geom)
        self.node = aspect2d.attachNewNode(node)
        self.node.setTwoSided(True)
        self.node.setColor(color4)
        self.node.setTransparency(TransparencyAttrib.MAlpha)

        # val = 987
        # self.text = TextNode("text")
        # self.text.setFont(get_B612_font())
        # self.text.setText(self.format % val)
        # self.text.setAlign(TextNode.ACenter)
        # self.text.setTextColor((1, 1, 1, 1))
        # self.node = aspect2d.attachNewNode(self.text)
        # self.node.setScale(size)
        # self.node.setBin("fixed", 1)
