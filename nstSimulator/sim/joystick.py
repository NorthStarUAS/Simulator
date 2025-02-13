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
    def __init__(self, joysticks_user={}):
        self.mapping = {}

        # internally tracked trim values (move out of here, and up a level?)
        self.yaw_trim = 0.0
        self.pitch_trim = 0.0

        if not have_pygame:
            return

        pygame.init()
        pygame.joystick.init()
        if not pygame.joystick.get_count():
            indent = ""
            for i in range(10):
                print(indent, "no joysticks found")
                indent += " "
                sleep(0.1)
            return
        print("Detected joysticks:", pygame.joystick.get_count())

        # load included joysticks config
        joysticks_path = Path(__file__).parent / "joysticks.json"
        print("loading joystick config file:", joysticks_path)
        f = open(joysticks_path, "r")
        joysticks_config = json.load(f)

        # append any upstream (aka user) joystick definitions if provided, these
        # will supercede any default definition for the same name joystick
        joysticks_config.extend(joysticks_user)

        # parse config for any sections that match a detected joystick
        for i in range(pygame.joystick.get_count()):
            handle = pygame.joystick.Joystick(i)
            name = handle.get_name()
            print("Detected joystick:", name, "axis:", handle.get_numaxes(), " buttons:", handle.get_numbuttons(), " hats:", handle.get_numhats())

            config_num = None
            for j, entry in enumerate(joysticks_config):
                if name in entry["names"]:
                    print("  Found config:", entry["names"])
                    config_num = j

            if config_num is not None:
                for key in joysticks_config[config_num]:
                    if key == "names":
                        continue
                    print(" ", key, joysticks_config[config_num][key])
                    vals = joysticks_config[config_num][key]
                    if len(vals) == 2:
                        self.mapping[key] = [ vals[0], i, vals[1] ]
                    elif len(vals) == 3:
                        self.mapping[key] = [ vals[0], i, vals[1], vals[2] ]
                    else:
                        print("wrong len of vals")
            else:
                print("  No config found.")
        print("Joystick mapping:", self.mapping)

    def deadband(self, x, val):
        if val < 0: val = 0
        if val > 0.9: val = 0.9
        if x >= val: return (x - val) / (1 - val)
        elif x <= -val: return (x + val) / (1 - val)
        else: return 0.0

    # reference: https://www.rcgroups.com/forums/showthread.php?375044-what-is-the-formula-for-the-expo-function
    def expo(self, x, val):
        # print(x, val, x * exp(abs(val*x))/exp(val))
        return x * exp(abs(val*x))/exp(val)

    def debug(self):
        # display all the raw joystick input data
        for i in range(pygame.joystick.get_count()):
            handle = pygame.joystick.Joystick(i)
            print(handle.get_name())
            print("  axes: ", end="")
            for i in range(handle.get_numaxes()):
                print("%d: %.2f " % (i, handle.get_axis(i)), end="")
            print()
            print("  buttons: ", end="")
            for i in range(handle.get_numbuttons()):
                print("%d: %d " % (i, handle.get_button(i)), end="")
            print()
            print("  hats: ", end="")
            for i in range(handle.get_numhats()):
                print("%d: " % i, handle.get_hat(i), end="")
            print()

    def update(self):
        if not pygame.joystick.get_count():
            return

        pygame.event.pump()
        # self.debug()

        # for each configuration mapping, fetch and process the value and store
        # it in inceptors_node with the same config name.
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
                    value = (value + 1) * 0.5
                inceptors_node.setDouble(name, value)
            elif input_type == "button":
                value = handle.get_button(element_num)
                inceptors_node.setBool(name, value)
            elif input_type == "hat":
                value = handle.get_hat(element_num)
                inceptors_node.setInt(name, value[0], 0)
                inceptors_node.setInt(name, value[1], 1)
        # inceptors_node.pretty_print()

        # Special handling of trim (for now, but should really move to a higher
        # level. Trim might mean different things to different airplanes or
        # flight control laws.)
        trim_cmd = 0
        trim_cmd += inceptors_node.getDouble("pitch_trim_down")
        trim_cmd -= inceptors_node.getDouble("pitch_trim_up")
        self.pitch_trim += 0.001 * trim_cmd
        if self.pitch_trim < -0.25: self.pitch_trim = -0.25
        if self.pitch_trim > 0.25: self.pitch_trim = 0.25
        # print("pitch trim:", self.pitch_trim)
        inceptors_node.setDouble("pitch_trim", self.pitch_trim)
