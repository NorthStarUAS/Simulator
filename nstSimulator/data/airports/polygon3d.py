# https://discourse.panda3d.org/t/polygon-drawer/12919

import numpy as np
from panda3d.core import *

# Create a fixed-color 2d polygon intended to be drawn on the aspect2d rendering
# layer.

class Polygon3d(object):
    def __init__(self, vertices=[], color=[1, 1, 1, 1]):
        # make local copy
        self.color = list(color)

        # sanitize and make local copy
        self.vertices = []
        p1 = np.array(vertices[0])
        self.vertices.append(p1)
        length = len(vertices)
        for i in range(1, length):
            p2 = np.array(vertices[i])
            dist = np.linalg.norm(p1[:2]-p2[:2])
            if dist > 0.01:
                self.vertices.append(p2)
                p1 = p2
            else:
                print("skipping:", dist, p1, p2)
        print("sanitize:", len(vertices), "->", len(self.vertices))

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
        print("vt:", len(self.vertices), "tris:", t.getNumTriangles())
        if t.getNumTriangles() > 3 * len(self.vertices):
            print("odd case check:")
            for i in range(len(vt)-1):
                p1 = np.array(vt[i][:2])
                p2 = np.array(vt[i+1][:2])
                dist = np.linalg.norm(p1-p2)
                if dist < 1:
                    print(dist, p1, p2)

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

