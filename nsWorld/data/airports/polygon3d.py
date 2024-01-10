# https://discourse.panda3d.org/t/polygon-drawer/12919

from panda3d.core import *

# Create a fixed-color 2d polygon intended to be drawn on the aspect2d rendering
# layer.

class Polygon3d(object):
    def __init__(self, vertices=[], color=[1, 1, 1, 1]):
        # make local copies
        self.vertices = list(vertices)
        self.color = list(color)

    def makeNode(self):
        vt = self.vertices
        vc = self.color
        t = Triangulator()
        format = GeomVertexFormat.getV3c4()
        vdata = GeomVertexData('name', format, Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        color = GeomVertexWriter(vdata, 'color')
        for x,y,z in vt:
            t.addPolygonVertex(t.addVertex(x,y))
            vertex.addData3(x, y, z)
            color.addData4(vc[0], vc[1], vc[2], vc[3])
        t.triangulate()
        prim = GeomTriangles(Geom.UHStatic)
        for n in range(t.getNumTriangles()):
            prim.addVertices(t.getTriangleV0(n), t.getTriangleV1(n), t.getTriangleV2(n))
        prim.closePrimitive()
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode('gnode')
        node.addGeom(geom)
        return node

# simple demonstration
# poly=Polygon([(0,0),(0,1),(1,1),(1,0)])
# import direct.directbase.DirectStart
# nodePath = render.attachNewNode(poly.makeNode())
# run()

