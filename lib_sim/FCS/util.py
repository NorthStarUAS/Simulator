# flying vs on ground detection.  Uses a sigmoid function between min/max
# threshold and compute a 0 - 1 likelihood.
from math import exp
class IsFlying():
    def __init__(self, on_ground_for_sure_mps, flying_for_sure_mps):
        self.on_ground_for_sure_mps = on_ground_for_sure_mps
        self.flying_for_sure_mps = flying_for_sure_mps
        self.diff = self.flying_for_sure_mps - self.on_ground_for_sure_mps

    def get_flying_confidence(self, vc_mps):
        diff = self.flying_for_sure_mps - self.on_ground_for_sure_mps
        # sigmoid function of [-5 to 5]
        x = 10 * (vc_mps - self.on_ground_for_sure_mps) / diff - 5
        flying_confidence = exp(x) / (1 + exp(x))
        print("flying:", "%.1f %.0f%%" % (vc_mps, 100*flying_confidence))
        return flying_confidence
