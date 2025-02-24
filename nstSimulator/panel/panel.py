from .guage import Guage

class Panel():
    def __init__(self):
        self.guage = Guage()

    def update(self):
        self.guage.update()