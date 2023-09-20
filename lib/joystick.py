# a simple locally connected (pc) joystick interface

from time import sleep

have_pygame = False
try:
    import pygame
    have_pygame = True
except:
    print("pygame import failed, joystick inactive")
    print("consider: dnf install python3-pygame")
    print("or pip install pygame")
    sleep(5)

class Joystick():
    def __init__(self):
        # physical values
        self.num_joysticks = 0
        self.joys = []
        self.mapping = {
            "aileron": [None] * 3, 
            "elevator": [None] * 3, 
            "elevator_trim": [None] * 3, 
            "rudder": [None] * 3, 
            "throttle": [None] * 3, 
        }

        # logical values
        self.throttle = 0.0
        self.aileron = 0.0
        self.rudder_trim = 0.0
        self.elevator = 0.0
        self.elevator_trim = 0.0
        self.rudder = 0.0

        if not have_pygame:
            return

        pygame.init()
        pygame.joystick.init()
        self.num_joysticks = pygame.joystick.get_count()
        if not self.num_joysticks:
            print("no joysticks found")
            sleep(5)
            return
        print("Detected joysticks:", pygame.joystick.get_count())

        for i in range(self.num_joysticks):
            name = pygame.joystick.Joystick(i).get_name()
            handle = pygame.joystick.Joystick(i)
            print("  name:", name)
            joy = {}
            joy["name"] = name
            joy["handle"] = handle
            joy["handle"].init()
            joy["num_axes"] = handle.get_numaxes()
            joy["axes"] = [0.0] * joy["num_axes"]
            joy["num_buttons"] = handle.get_numbuttons()
            joy["buttons"] = [0] * joy["num_buttons"]
            joy["num_hats"] = handle.get_numhats()
            joy["hats"] = [0] * joy["num_hats"]
            self.joys.append(joy)
            if name == "CLSE Joystick Infinity":
                self.mapping["aileron"] = ["axis", i, 0]
                self.mapping["elevator"] = ["axis", i, 1]
                self.mapping["elevator_trim"] = ["hat", 0, [0,1]]
            elif name == "TWCS Throttle":
                self.mapping["throttle"] = ["axis", i, 2]
                self.mapping["rudder"] = ["axis", i, 7]
        print("Joystick structures:", self.joys)
        print("Joystick mapping:", self.mapping)

    def get_joy_value(self, mapping):
        source, joy_num, axis_num = mapping
        val = 0.0
        if source is not None:
            if source == "axis":
                val = self.joys[joy_num]["axes"][axis_num]
            elif source == "hat":
                val = self.joys[joy_num]["hats"][axis_num[0]][axis_num[1]]
        return val

    def update(self):
        if not self.num_joysticks:
            return

        pygame.event.pump()

        for joy in self.joys:
            handle = joy["handle"]
            for i in range(joy["num_axes"]):
                joy["axes"][i] = handle.get_axis(i)
            for i in range(joy["num_buttons"]):
                joy["buttons"][i] = handle.get_button(i)
            for i in range(joy["num_hats"]):
                joy["hats"][i] = handle.get_hat(i)
            # print(joy)

        self.throttle = (1.0 - self.get_joy_value(self.mapping["throttle"])) * 0.5
        # if self.num_buttons >= 6:
        #     if self.buttons[5]:
        #         self.elevator_trim -= 0.001
        #     elif self.buttons[4]:
        #         self.elevator_trim += 0.001
        #     if self.elevator_trim < -0.25: self.elevator_trim = -0.25
        #     if self.elevator_trim >  0.25: self.elevator_trim =  0.25
        # if self.num_buttons >= 12:
        #     if self.buttons[11]:
        #         self.rudder_trim -= 0.001
        #     elif self.buttons[10]:
        #         self.rudder_trim += 0.001
        #     if self.rudder_trim < -0.25: self.rudder_trim = -0.25
        #     if self.rudder_trim >  0.25: self.rudder_trim =  0.25
        if False and self.num_hats:
            self.elevator_trim += self.hats[0][1] * 0.001
            if self.elevator_trim < -0.25: self.elevator_trim = -0.25
            if self.elevator_trim >  0.25: self.elevator_trim =  0.25
        self.aileron = self.get_joy_value(self.mapping["aileron"])
        self.elevator_trim += 0.001 * self.get_joy_value(self.mapping["elevator_trim"])
        print("elevator trim:", self.elevator_trim)
        self.elevator = -self.get_joy_value(self.mapping["elevator"]) + self.elevator_trim
        self.rudder = self.get_joy_value(self.mapping["rudder"]) + self.rudder_trim
