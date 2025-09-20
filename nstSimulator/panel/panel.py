from .guage import Guage
from .pitch import PitchLadderF16
from .speed import SpeedTapeYukikaze

class Panel():
    def __init__(self):
        self.guage = Guage(0.25, 0, 0)
        self.pitch = PitchLadderF16()
        self.speed = SpeedTapeYukikaze()

    def update(self, nedpos):
        self.guage.update()
        self.pitch.update(nedpos)
        self.speed.update(nedpos)