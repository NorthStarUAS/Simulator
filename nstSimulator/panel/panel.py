from .guage import Guage
from .pitch import PitchLadderF16

class Panel():
    def __init__(self):
        self.guage = Guage(0.25, 0, 0)
        self.pitch = PitchLadderF16()

    def update(self, nedpos):
        self.guage.update()
        self.pitch.update(nedpos)