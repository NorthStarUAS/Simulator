# Just to be clear: this is not a PID implementation!

# This a very specialized supporting class that implments two helper functions:
#
# 1. A complex function to take a command request as input.  It condition the
#    cmd against limits and when the command is 0, holds the separately provide
#    target value.  This can be used to implement a rate-command, position-hold
#    system.
#
# 2. An integrator.  The reality is the up-stream controller model is not
#    perfect so this class offers an integrator (essentially the "I" from a PID)
#    to suck up all the unaccounted for errors.  In addition the up-stream
#    controller can set a very conservative anti-windup threshold to limit the
#    overall effectiveness of the integrator (so pilot would win a force fight)
#    and as an added bonus, the integrator is feathered off/on based on the
#    value of flying_confidence (0-1) which the up-stream controller also is
#    responsible for providing.
class NotaPID():
    def __init__(self, name, min_hold, max_hold, integral_gain, antiwindup, neutral_tolerance):
        self.dt = 0.02
        self.name = name
        self.int_gain = integral_gain
        self.antiwindup = antiwindup
        self.tol = neutral_tolerance
        self.cmd_neutral = True
        self.min_hold = min_hold
        self.max_hold = max_hold
        self.hold_cmd = 0.0
        self.error_sum = 0.0

    def get_ref_value(self, input_cmd, ff_cmd, min_val, max_val, cur_val, flying_confidence):
        if flying_confidence < 0.01:
            self.hold_cmd = cur_val
        if abs(input_cmd) < self.tol:
            if not self.cmd_neutral:
                # print("set neutral:", self.name)
                self.hold_cmd = cur_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False
        if self.hold_cmd < self.min_hold:
            self.hold_cmd = self.min_hold
        if self.hold_cmd > self.max_hold:
            self.hold_cmd = self.max_hold
        if self.cmd_neutral:
            error = (self.hold_cmd - cur_val) * flying_confidence
            ref_val = error * 0.05 + ff_cmd
            # print(self.name, ref_rate)
        else:
            ref_val = input_cmd + ff_cmd

        if max_val is not None and ref_val > max_val:
            ref_val = max_val
        if min_val is not None and ref_val < min_val:
            ref_val = min_val
        return ref_val

    def integrator(self, ref_val, cur_val, flying_confidence=0.0):
        cutoff = self.antiwindup * flying_confidence
        self.error_sum += self.int_gain * (ref_val - cur_val) * self.dt
        if self.error_sum < -cutoff: self.error_sum = -cutoff
        if self.error_sum > cutoff: self.error_sum = cutoff
        # print(self.name, "ref_val: %.2f" % ref_val, "error sum: %.2f" % self.error_sum, "%s: %.2f" % (self.name, self.error_sum * self.int_gain))
        return self.error_sum

# flying vs on ground detection.  Uses a sigmoid function between min/max
# threshold and compute a 0 - 1 likelihood.
from math import exp
class IsFlying():
    def __init__(self, on_ground_for_sure_mps, flying_for_sure_mps):
        self.on_ground_for_sure_mps = on_ground_for_sure_mps
        self.flying_for_sure_mps = flying_for_sure_mps
        self.diff = self.flying_for_sure_mps - self.on_ground_for_sure_mps

    def get_flying_confidence(self, vc_mps):
        diff = self.flying_for_sure_mps - self.on_ground_for_sure_mps
        # sigmoid function of [-5 to 5]
        x = 10 * (vc_mps - self.on_ground_for_sure_mps) / diff - 5
        flying_confidence = exp(x) / (1 + exp(x))
        print("flying:", "%.1f %.0f%%" % (vc_mps, 100*flying_confidence))
        return flying_confidence
