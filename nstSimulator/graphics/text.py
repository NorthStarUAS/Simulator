from panda3d.core import *

from nstSimulator.graphics.fonts import get_B612_font

class Text3d():
    def __init__(self, color=(0.9, 0.1, 0.1, 0.7), align=TextNode.ACenter, scale=1, val=0.0, format="%.0f kts", pad=1):
        self.scale = scale
        self.format = format

        self.node = NodePath("text")

        self.text = TextNode("text")
        self.text.setFont(get_B612_font())
        self.text.setText(self.format % val)
        self.text.setAlign(align)
        self.text.setTextColor(color)
        self.text.setTextScale(scale)
        self.text.setCardAsMargin(pad, pad, pad, pad)
        self.text.setCardColor(0, 0, 0, 1)
        self.text.setCardDecal(True)
        self.text.setFrameAsMargin(pad, pad, pad, pad)
        self.text.setFrameColor(color)
        self.node.attachNewNode(self.text)
        # self.node = aspect2d.attachNewNode(self.text)

        # bounds = self.node.getTightBounds()
        # print("bounds:", bounds)
        # xsize = bounds[1][0] - bounds[0][0]
        # ysize = bounds[1][2] - bounds[0][2]
        # # print("ysize:", ysize)
        # self.node.setPos(center_x, 0, center_y - 0.43*ysize)

        # cm = CardMaker('background')
        # cm.setFrame(bounds)
        # cm.setColor(0, 0, 0, 0.2)
        # self.background = aspect2d.attachNewNode(cm.generate())
        # self.background.setTransparency(TransparencyAttrib.MAlpha)

    def update(self, val):
        self.text.setText(self.format % val)
