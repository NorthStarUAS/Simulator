from panda3d.core import *

class LightMgr():
    def __init__(self, config):
        # Light settings
        if "ambient" in config:
            self.ambientLight = render.attachNewNode(AmbientLight("ambientLight"))
            self.ambientLight.node().setColor(config["ambient"])
            render.setLight(self.ambientLight)

        if "sun" in config:
            self.sun = render.attachNewNode(DirectionalLight("sun"))
            self.sun.node().setColor(config["sun"])
            v = LVector3(0.2, 0.2, -0.8)
            if "sun_vector" in config:
                v = LVector3(config["sun_vector"][0], config["sun_vector"][1], config["sun_vector"][2])
            self.sun.node().setDirection(v)
            render.setLight(self.sun)

    def update(self):
        pass