# This a very specialized supporting class that implments a helper function:
#
# Takes a command request as input.  It conditions the cmd against limits and
# when the command is 0, holds the separately provide target value.  When the
# system is in hold mode, the command is the output of a basic "P" controller
# implicitely implimented here.
#
# The hold limits set the range of the hold command.  When the inceptor is
# centered (hold) the aircraft will return to within this range.  The function
# could be configured so it would be possible to command reference value that
# would take the aircraft outside this range (while under active control.)
#
# The ref limits set the range of the actual reference command. (ex: max roll
# rate or max load factor.)
#
# Note that the feed forward command is included in the math, so that the ref
# limits can be applied /after/ the feed forward is added to avoid over
# stressing the airframe.

class HoldOrPassThrough():
    def __init__(self, name, min_hold_limit, max_hold_limit, min_ref_limit, max_ref_limit, neutral_tolerance, hold_gain=0.1, debug=False):
        self.name = name
        self.tol = neutral_tolerance
        self.cmd_neutral = True
        self.min_hold_limit = min_hold_limit
        self.max_hold_limit = max_hold_limit
        self.min_ref_limit = min_ref_limit
        self.max_ref_limit = max_ref_limit
        self.hold_cmd = 0.0
        self.error_sum = 0.0
        self.hold_gain = hold_gain
        self.debug = debug

    def get_ref_value(self, input_cmd, ff_cmd, cur_val, flying_confidence):
        if flying_confidence < 0.01:
            self.hold_cmd = cur_val

        if abs(input_cmd) < self.tol:
            if not self.cmd_neutral:
                # print("set neutral:", self.name)
                self.hold_cmd = cur_val
                self.cmd_neutral = True
        else:
            self.cmd_neutral = False

        if self.hold_cmd < self.min_hold_limit: self.hold_cmd = self.min_hold_limit
        if self.hold_cmd > self.max_hold_limit: self.hold_cmd = self.max_hold_limit

        if self.cmd_neutral:
            hold_error = (self.hold_cmd - cur_val) * flying_confidence
            if self.debug: print("hold_cmd: %.2f" % self.hold_cmd, "cur_val: %.2f" % cur_val)
            if self.debug: print("hold_error = %.2f" % hold_error)
            ref_val = hold_error * self.hold_gain
            # print(self.name, ref_rate)
        else:
            ref_val = input_cmd

        ref_val += ff_cmd
        if ref_val > self.max_ref_limit: ref_val = self.max_ref_limit
        if ref_val < self.min_ref_limit: ref_val = self.min_ref_limit

        # if max_ref_limit is not None and ref_val > max_ref_limit: ref_val = max_ref_limit
        # if min_ref_limit is not None and ref_val < min_ref_limit: ref_val = min_ref_limit

        return ref_val

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
