# a simple locally connected (pc) joystick interface

have_pygame = False
try:
    import pygame
    have_pygame = True
except:
    print("pygame import failed, joystick inactive")

class Joystick():
    def __init__(self):
        # physical values
        self.have_joystick = False
        self.num_axes = 0
        self.num_buttons = 0
        self.num_hats = 0
        self.axes = []
        self.buttons = []
        self.hats = []

        # logical values
        self.throttle = 0.0
        self.aileron = 0.0
        self.elevator = 0.0
        self.elevator_trim = 0.0
        self.rudder = 0.0
        
        if have_pygame:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.have_joystick = True
                print("Detected a joystick")
            else:
                print("no joysticks found")
        
        if self.have_joystick:
            self.j = pygame.joystick.Joystick(0)
            self.j.init()
            self.num_axes = self.j.get_numaxes()
            self.axes = [0.0] * self.num_axes
            self.num_buttons = self.j.get_numbuttons()
            self.buttons = [0] * self.num_buttons
            self.num_hats = self.j.get_numhats()
            self.hats = [0] * self.num_hats

    def update(self):
        if not self.have_joystick:
            return
        
        pygame.event.pump()
        
        for i in range(self.num_axes):
            self.axes[i] = self.j.get_axis(i)
        for i in range(self.num_buttons):
            self.buttons[i] = self.j.get_button(i)
        for i in range(self.num_hats):
            self.hats[i] = self.j.get_hat(i)
        print(self.axes, self.buttons, self.hats)
        
        self.throttle = (1.0 - self.axes[2]) * 0.5
        self.aileron = self.axes[0]
        if self.num_hats:
            self.elevator_trim += self.hats[0][1] * 0.001
            if self.elevator_trim < -0.25: self.elevator_trim = -0.25
            if self.elevator_trim >  0.25: self.elevator_trim =  0.25
        self.elevator = -self.axes[1] + self.elevator_trim
        self.rudder = self.axes[3]
