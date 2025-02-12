# a simple locally connected (pc) joystick interface

import json
from math import exp
from pathlib import Path
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

from .lib.props import inceptors_node

class Joystick():
    def __init__(self, joysticks_config=None):
        self.mapping = {}

        # internally tracked trim values
        self.yaw_trim = 0.0
        self.pitch_trim = 0.0

        if not have_pygame:
            return

        pygame.init()
        pygame.joystick.init()
        if not pygame.joystick.get_count():
            for i in range(10):
                print("no joysticks found")
                sleep(0.1)
            return
        print("Detected joysticks:", pygame.joystick.get_count())

        # load joysticks config
        if joysticks_config is None:
            joysticks_path = Path(__file__).parent / "joysticks.json"
            print("loading joystick config file:", joysticks_path)
            f = open(joysticks_path, "r")
            joysticks_config = json.load(f)

        for i in range(pygame.joystick.get_count()):
            name = pygame.joystick.Joystick(i).get_name()
            print("Joystick:", name, "- checking for a config:")

            if name in joysticks_config:
                for key in joysticks_config[name]:
                    print(" ", key, joysticks_config[name][key])
                    vals = joysticks_config[name][key]
                    if len(vals) == 2:
                        self.mapping[key] = [ vals[0], i, vals[1] ]
                    elif len(vals) == 3:
                        self.mapping[key] = [ vals[0], i, vals[1], vals[2] ]
                    else:
                        print("wrong len of vals")

        print("Joystick mapping:", self.mapping)

    def deadband(self, x, val):
        if val < 0: val = 0
        if val > 0.9: val = 0.9
        if False:
            if x >= val: result = (x - val) / (1 - val)
            elif x <= -val: result = (x + val) / (1 - val)
            else: result = 0.0
            print("deadband:", x, "->", result)
        if x >= val: return (x - val) / (1 - val)
        elif x <= -val: return (x + val) / (1 - val)
        else: return 0.0

    # reference: https://www.rcgroups.com/forums/showthread.php?375044-what-is-the-formula-for-the-expo-function
    def expo(self, x, val):
        # print(x, val, x * exp(abs(val*x))/exp(val))
        return x * exp(abs(val*x))/exp(val)

    # this needs to get rewritten to access the data directly
    def debug(self):
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

    def update(self):
        if not pygame.joystick.get_count():
            return

        pygame.event.pump()


        for name in self.mapping.keys():
            if len(self.mapping[name]) == 3:
                [input_type, joy_num, element_num] = self.mapping[name]
                filters = {}
            elif len(self.mapping[name]) == 4:
                [input_type, joy_num, element_num, filters] = self.mapping[name]
            handle = pygame.joystick.Joystick(joy_num)
            if input_type == "axis":
                value = handle.get_axis(element_num)
                if "deadband" in filters:
                    value = self.deadband(value, filters["deadband"])
                if "expo" in filters:
                    value = self.expo(value, filters["expo"])
                if "reverse" in filters:
                    value = -value
                if "normalize_0_1" in filters:
                    # print("raw power:", value)
                    value = (value + 1) * 0.5
                inceptors_node.setDouble(name, value)
            elif input_type == "button":
                value = handle.get_button(element_num)
                inceptors_node.setBool(name, value)
            elif input_type == "hat":
                value = handle.get_hat(element_num)
                inceptors_node.setBool(name, value)
            # print("  name:", name, "val:", value)

        trim_cmd = 0
        trim_cmd += inceptors_node.getDouble("pitch_trim_down")
        trim_cmd -= inceptors_node.getDouble("pitch_trim_up")
        self.pitch_trim += 0.001 * trim_cmd
        if self.pitch_trim < -0.25: self.pitch_trim = -0.25
        if self.pitch_trim > 0.25: self.pitch_trim = 0.25
        # print("pitch trim:", self.pitch_trim)
        inceptors_node.setDouble("pitch_trim", self.pitch_trim)
