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
    def __init__(self, config_path):
        self.do_reset = True
        self.y_n = 0.0
        self.y_n_1 = 0.0
        self.iterm = 0.0

        self.component_node = PropertyNode( config_path )

        # enable
        enable_prop = self.component_node.getString("enable")
        pos = enable_prop.rfind("/")
        if pos >= 0:
            self.enable_node = PropertyNode(enable_prop[:pos])
            self.enable_attr = enable_prop[pos+1:]
            print("enable: %s / %s" % (enable_prop[:pos], self.enable_attr))

        # input
        input_prop = self.component_node.getString("input")
        pos = input_prop.rfind("/")
        if pos >= 0:
            self.input_node = PropertyNode(input_prop[:pos])
            self.input_attr = input_prop[pos+1:]
            print("input: %s / %s" % (input_prop[:pos], self.input_attr))

        # feed forward term (computed externally)
        ff_prop = self.component_node.getString("feed_forward")
        pos = ff_prop.rfind("/")
        if pos >= 0:
            self.ff_node = PropertyNode(ff_prop[:pos])
            self.ff_attr = ff_prop[pos+1:]
            print("feed forward: %s / %s" % (ff_prop[:pos], self.ff_attr))

        # reference
        ref_prop = self.component_node.getString("reference")
        pos = ref_prop.rfind("/")
        if pos >= 0:
            self.ref_node = PropertyNode(ref_prop[:pos])
            self.ref_attr = ref_prop[pos+1:]
            print("ref: %s / %s" % (ref_prop[:pos], self.ref_attr))

        # output
        output_prop = self.component_node.getString("output")
        pos = output_prop.rfind("/")
        if pos >= 0:
            self.output_node = PropertyNode(output_prop[:pos])
            self.output_attr = output_prop[pos+1:]
            print("output: %s / %s" % (output_prop[:pos], self.output_attr))

        # config
        self.config_node = self.component_node.getChild( "config" )

    def reset(self):
        self.do_reset = True

    def update(self, dt):
        if len(self.enable_attr):
            enabled = self.enable_node.getBool(self.enable_attr)
        else:
            enabled = True

        debug = self.component_node.getBool("debug")
        if debug: print("Updating:", self.component_node.getString("name"))
        y_n = self.input_node.getDouble(self.input_attr)
        r_n = self.ref_node.getDouble(self.ref_attr)

        error = r_n - y_n

        wrap = self.component_node.getString("wrap")
        if wrap == "180":
            # wrap error (by +/- 360 degrees to put the result in [-180, 180]
            if error < -180: error += 360
            if error > 180: error -= 360
        elif wrap == "pi":
            # wrap error (by +/- 2*pi degrees to put the result in [-pi, pi]
            if error < -pi: error += 2*pi
            if error > pi: error -= 2*pi

        if debug: print("input = %.3f reference = %.3f error = %.3f" % (y_n, r_n, error))

        u_min = self.config_node.getDouble("u_min")
        u_max = self.config_node.getDouble("u_max")

        Kp = self.config_node.getDouble("Kp")
        Ti = self.config_node.getDouble("Ti")
        Td = self.config_node.getDouble("Td")
        if Ti > 0.0001:
            Ki = Kp / Ti
        else:
            Ki = 0.0
        Kd = Kp * Td

        # feed forward term
        if len(self.ff_attr):
            ffterm = self.ff_node.getDouble(self.ff_attr)
        else:
            ffterm = 0.0

        # proportional term (include the feed forward term here to fascilitate
        # reset-transient avoidance and anti-windup)
        pterm = Kp * error + ffterm

        # integral term
        if Ti > 0.0001:
            iterm += Ki * error * dt
        else:
            iterm = 0.0

        # if the reset flag is set, back compute an iterm that will produce zero
        # initial transient (overwriting the existing iterm) then unset the
        # do_reset flag.
        if do_reset:
            if Ti > 0.0001:
                u_n = self.output_node.getDouble(self.output_attr)
                # and clip
                if u_n < u_min: u_n = u_min
                if u_n > u_max: u_n = u_max
                iterm = u_n - pterm
            else:
                iterm = 0.0
            do_reset = False

        # derivative term: observe that dError/dt = -dInput/dt (except when the
        # setpoint changes (which we don't want to react to anyway.)  This
        # approach avoids "derivative kick" when the set point changes.
        dy = y_n - y_n_1
        y_n_1 = y_n
        dterm = Kd * -dy / dt

        # anti-integrator windup
        output = pterm + iterm + dterm
        if output < u_min:
            if Ti > 0.0001:
                iterm += u_min - output
            output = u_min
        if output > u_max:
            if Ti > 0.0001:
                iterm -= output - u_max
            output = u_max

        if debug: print("pterm = %.3f iterm = %.3f ffterm = %.3f" % (pterm, iterm, ffterm))

        if not enabled:
            # this will force a reset when component becomes enabled
            do_reset = True
        else:
            self.output_node.setDouble( self.output_attr, output )
