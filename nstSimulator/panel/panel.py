from .altittude import AltitudeTapeYukikaze
from .pitch import PitchLadderF16
from .speed import SpeedTapeYukikaze

class Panel():
    def __init__(self):
        self.pitch = PitchLadderF16()
        self.alt = AltitudeTapeYukikaze()
        self.speed = SpeedTapeYukikaze()

    def update(self, nedpos):
        self.pitch.update(nedpos)
        self.alt.update(nedpos)
        self.speed.update(nedpos)