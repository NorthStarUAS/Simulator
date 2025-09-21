from .altittude import AltitudeTapeYukikaze
from .heading import HeadingTapeYukikaze
from .pitch import PitchLadderF16, PitchLadderYukikaze
from .speed import SpeedTapeYukikaze

class Panel():
    def __init__(self):
        # self.pitch = PitchLadderF16()
        self.pitch = PitchLadderYukikaze()
        self.alt = AltitudeTapeYukikaze()
        self.heading = HeadingTapeYukikaze()
        self.speed = SpeedTapeYukikaze()

    def update(self, nedpos):
        self.pitch.update(nedpos)
        self.alt.update(nedpos)
        self.heading.update(nedpos)
        self.speed.update(nedpos)