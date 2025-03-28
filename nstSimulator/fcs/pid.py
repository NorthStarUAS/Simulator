# pid.py - a fairly standard pid controller
#
# Written by Curtis Olson, started January 2004.
#
# Copyright (C) 2004-2025 Curtis L. Olson - curtolson@flightgear.org
#
# <MIT license here>

from math import pi

from PropertyTree import PropertyNode

class ap_pid_t:
    def __init__(self, name):
        self.name = name
        self.do_reset = True
        self.enable = False
        self.debug = False
        self.wrap = "none"
        self.u_min = -1
        self.u_max = 1
        self.Kp = 1
        self.Ti = 0
        self.Td = 0
        self.y_n = 0
        self.y_n_1 = 0
        self.iterm = 0
        self.max_integrator = -1
        self.output = 0
        self.reset_output_val = 0

    def reset(self, output_val):
        self.do_reset = True
        self.reset_output_val = output_val

    # y = input value (current sensor value)
    # r = reference value (target/goal value)
    def update(self, dt, y, r):
        if self.debug: print("Updating:", self.name)
        self.y_n = y
        error = r - self.y_n

        if self.wrap == "180":
            # wrap error (by +/- 360 degrees to put the result in [-180, 180]
            if error < -180: error += 360
            if error > 180: error -= 360
        elif self.wrap == "pi":
            # wrap error (by +/- 2*pi degrees to put the result in [-pi, pi]
            if error < -pi: error += 2*pi
            if error > pi: error -= 2*pi

        if self.debug: print("input = %.3f reference = %.3f error = %.3f" % (self.y_n, r, error))

        if self.Ti > 0.0001:
            Ki = self.Kp / self.Ti
        else:
            Ki = 0.0
        Kd = self.Kp * self.Td

        # proportional term (include the feed forward term here to fascilitate
        # reset-transient avoidance and anti-windup)
        pterm = self.Kp * error

        # integral term
        if self.Ti > 0.0001:
            if not self.do_reset:
                self.iterm += Ki * error * dt
            else:
                # if the reset flag is set, back compute an iterm that will
                # produce zero output transient (overwriting the existing iterm)
                # then unset the do_reset flag.
                print("resetting u_n:", self.reset_output_val)
                self.do_reset = False
                self.y_n_1 = self.y_n  # so first dy isn't crazy
                u_n = self.reset_output_val
                if u_n < self.u_min: u_n = self.u_min
                if u_n > self.u_max: u_n = self.u_max
                self.iterm = u_n - pterm
                # print("  new iterm:", self.iterm)
        else:
            self.iterm = 0.0

        # derivative term: observe that dError/dt = -dInput/dt (except when the
        # setpoint changes (which we don't want to react to anyway.)  This
        # approach avoids "derivative kick" when the set point changes.
        dy = self.y_n - self.y_n_1
        self.y_n_1 = self.y_n
        dterm = Kd * -dy / dt
        if self.debug: print("  Kd: %.3f  dy: %.3f  dt: %.3f" % (Kd, dy, dt))

        self.output = pterm + self.iterm + dterm

        # output clipping and integrator windup protection
        if self.output < self.u_min:
            if self.Ti > 0.0001:
                self.iterm += self.u_min - self.output  # fixme: should this be computed without the dterm?
            self.output = self.u_min
        if self.output > self.u_max:
            if self.Ti > 0.0001:
                self.iterm -= self.output - self.u_max  # fixme: should this be computed without the dterm?
            self.output = self.u_max

        # max integrator clipping (if specified)
        if self.max_integrator >= 0:
            if self.iterm < -self.max_integrator: self.iterm = -self.max_integrator
            if self.iterm > self.max_integrator: self.iterm = self.max_integrator

        if self.debug: print("pterm = %.3f + iterm = %.3f + dterm = %.3f = output = %.3f" % (pterm, self.iterm, dterm, self.output))

        if not self.enable:
            # this will force a reset when component becomes enabled
            self.do_reset = True
            return 0.0
        else:
            return self.output
