from .guage import Guage

class Panel():
    def __init__(self):
        self.guage = Guage(0.25, 0, 0)

    def update(self):
        self.guage.update()