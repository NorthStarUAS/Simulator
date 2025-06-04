from panda3d.core import *

class FogMgr():
    def __init__(self, config):
        # exponential fog

        # default, but change this in the top level config, not here!
        fog_color = (0.373, 0.369, 0.431)  # match twilight skybox
        if "color" in config:
            fog_color = config["color"]
        self.expfog = Fog("exponential fog")
        self.expfog.setColor(*fog_color)
        self.expfog.setExpDensity(0.00006)
        render.setFog(self.expfog)
        base.setBackgroundColor(*fog_color)

    def update(self, alt_m):
        alt_factor = alt_m / 10000
        if alt_factor > 1: alt_factor = 1
        fog_val = 0.00006 - (0.00002 * alt_factor)
        if fog_val < 0.00002: fog_val = 0.00002
        self.expfog.setExpDensity(fog_val)
