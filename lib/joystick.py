# a simple locally connected (pc) joystick interface

from math import exp
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

from lib.props import inceptor_node

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
            "flaps": [None] * 3,
            "gear": [None] * 3,
        }

        # internally tracked trim values
        self.rudder_trim = 0.0
        self.elevator_trim = 0.0

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
                self.mapping["elevator_trim"] = ["hat", i, 0, 1]
            elif name == "TWCS Throttle":
                self.mapping["throttle"] = ["axis", i, 2]
                self.mapping["rudder"] = ["axis", i, 7, {"expo": 2}]
                self.mapping["flaps_down"] = ["button", i, 4]
                self.mapping["flaps_up"] = ["button", i, 3]
            elif name == "Thrustmaster T.16000M":
                self.mapping["aileron"] = ["axis", i, 0]
                self.mapping["elevator"] = ["axis", i, 1]
                self.mapping["rudder"] = ["axis", i, 2, {"expo": 2}]
                self.mapping["throttle"] = ["axis", i, 3]
                self.mapping["elevator_trim_down"] = ["button", i, 4]
                self.mapping["elevator_trim_up"] = ["button", i, 9]
                self.mapping["flaps_down"] = ["button", i, 15]
                self.mapping["flaps_up"] = ["button", i, 10]
            elif name == "Logitech Extreme 3D pro":
                self.mapping["aileron"] = ["axis", i, 0]
                self.mapping["elevator"] = ["axis", i, 1]
                self.mapping["rudder"] = ["axis", i, 2, {"expo": 2}]
                self.mapping["throttle"] = ["axis", i, 3]
                self.mapping["elevator_trim_down"] = ["button", i, 4]
                self.mapping["elevator_trim_up"] = ["button", i, 2]
                self.mapping["flaps_down"] = ["button", i, 3]
                self.mapping["flaps_up"] = ["button", i, 5]

        print("Joystick structures:", self.joys)
        print("Joystick mapping:", self.mapping)

    # reference: https://www.rcgroups.com/forums/showthread.php?375044-what-is-the-formula-for-the-expo-function
    def expo(self, x, val):
        # print(x, val, x * exp(abs(val*x))/exp(val))
        return x * exp(abs(val*x))/exp(val)

    def get_input_value(self, name):
        val = 0.0
        if name in self.mapping:
            mapping = self.mapping[name]
            source = mapping[0]
            joy_num = mapping[1]
            element_num = mapping[2]
            if source is not None:
                if source == "axis":
                    val = self.joys[joy_num]["axes"][element_num]
                    if len(mapping) > 3:
                        options = mapping[3]
                        if "expo" in options:
                            val = self.expo(val, options["expo"])
                elif source == "hat":
                    sub_num = mapping[3]
                    val = self.joys[joy_num]["hats"][element_num][sub_num]
                elif source == "button":
                    val = self.joys[joy_num]["buttons"][element_num]
                # if name == "elevator_trim_up":
                #     print(mapping, val)
        return val

    def update(self):
        if not self.num_joysticks:
            return

        pygame.event.pump()

        # read all the raw input data
        for joy in self.joys:
            handle = joy["handle"]
            for i in range(joy["num_axes"]):
                joy["axes"][i] = handle.get_axis(i)
            # print(joy["axes"])
            for i in range(joy["num_buttons"]):
                joy["buttons"][i] = handle.get_button(i)
            # print(joy["buttons"])
            for i in range(joy["num_hats"]):
                joy["hats"][i] = handle.get_hat(i)
            # print(joy)

        inceptor_node.setFloat("throttle", (1.0 - self.get_input_value("throttle")) * 0.5)

        inceptor_node.setFloat("aileron", self.get_input_value("aileron"))

        trim_cmd = 0
        trim_cmd += self.get_input_value("elevator_trim_down")
        trim_cmd -= self.get_input_value("elevator_trim_up")
        # trim_cmd += self.get_input_value("elevator_trim")
        self.elevator_trim += 0.001 * trim_cmd
        if self.elevator_trim < -0.25: self.elevator_trim = -0.25
        if self.elevator_trim > 0.25: self.elevator_trim = 0.25
        # print("elevator trim:", self.elevator_trim)
        inceptor_node.setFloat("elevator_trim", self.elevator_trim)

        inceptor_node.setFloat("elevator", -self.get_input_value("elevator"))

        inceptor_node.setFloat("rudder", self.get_input_value("rudder"))

        inceptor_node.setBool("flaps_down",  self.get_input_value("flaps_down"))
        inceptor_node.setBool("flaps_up",  self.get_input_value("flaps_up"))
        # print(inceptor_node.getBool("flaps_down"), inceptor_node.getBool("flaps_up"))
