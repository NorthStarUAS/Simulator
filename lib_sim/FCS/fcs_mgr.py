from math import acos, cos, pi, sin, tan

from nstSimulator.utils.constants import d2r, g, r2d
from nstSimulator.sim.lib.props import aero_node, att_node, control_node, fcs_node, imu_node, inceptors_node, vel_node
from PropertyTree import PropertyNode

from .inertial_airdata_est import alpha_func, beta_func, inv_alpha_func
from .roll_controller import p_controller
from .nzthe_controller import nzthe_controller
# from .nzu_controller import nzu_controller
from .yaw_controller import r_controller
from .util import IsFlying

rho = 1.225

class FCSMgr():
    def __init__(self):
        # stick -> rate command scaling
        self.roll_stick_scale = 40 * d2r
        self.lf_stick_scale = -1.5
        self.yaw_stick_scale = -10 * d2r

        # alpha limit (est)
        self.alpha_limit_deg = 13

        self.fcs_lat = p_controller()
        # self.fcs_lon = q_controller()
        self.fcs_lon = nzthe_controller()
        # self.fcs_lon = nzu_controller()
        self.fcs_yaw = r_controller()
        self.is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=35)

        # filtered state (clamp to minimum of 25 mps because we need to divide
        # by airspeed and qbar so this must be definitely positive 100% of the time.)
        self.vc_filt_mps = 25
        self.vtrue_filt_mps = 25

        # fixme: this is a hack to put this here. But for now we start out with
        # manual/joystick flight control and if we receive remote (HIL) effector
        # commands we switch to external FCS control.  Would prefer a more
        # explicite way to do this in a better location.
        inceptors_node.setBool("master_switch", False)

    def compute_stuff(self):
        # update state and filters

        # velocity terms
        vc_mps = vel_node.getDouble("vc_mps")
        if vc_mps < 25: vc_mps = 25
        self.vc_filt_mps = 0.99 * self.vc_filt_mps + 0.01 * vc_mps
        vtrue_mps = vel_node.getDouble("vtrue_mps")
        if vtrue_mps < 25: vtrue_mps = 25
        self.vtrue_filt_mps = 0.99 * self.vtrue_filt_mps + 0.01 * vtrue_mps
        qbar = 0.5 * self.vc_filt_mps**2 * rho
        fcs_node.setDouble("vc_filt_mps", self.vc_filt_mps)
        fcs_node.setDouble("qbar", qbar)

        # alpha / beta estimates (or direct from sim model)
        if fcs_node.getDouble("flying_confidence") > 0.5:
            if False:
                # sensed directly (or from sim model)
                alpha_deg = aero_node.getDouble("alpha_deg")
                beta_deg = aero_node.getDouble("beta_deg")
            else:
                # inertial+airdata estimate fit from flight data
                rudder_cmd = inceptors_node.getDouble("yaw")
                throttle_cmd = inceptors_node.getDouble("power")
                flaps_norm = fcs_node.getDouble("posFlap_norm")
                az_mps2 = imu_node.getDouble("az_mps2")
                print("accels: %.2f %.2f %.2f" % (imu_node.getDouble("ax_mps2"), imu_node.getDouble("ay_mps2"), az_mps2))
                alpha_deg = alpha_func(flaps_norm, qbar, az_mps2)
                print("alpha true: %.1f  est: %.1f diff: %.1f" % (aero_node.getDouble("alpha_deg"), alpha_deg, aero_node.getDouble("alpha_deg") - alpha_deg))
                # beta_deg = beta_func(qbar, ay, r, rudder_cmd, throttle_cmd)  # this functions drifts and can get stuck!
                beta_deg = aero_node.getDouble("beta_deg")
        else:
            alpha_deg = att_node.getDouble("theta_deg")
            beta_deg = 0
        fcs_node.setDouble("alpha_deg", alpha_deg)
        fcs_node.setDouble("beta_deg", beta_deg)
        print("beta: %.1f" % beta_deg)

        # Feed forward steady state q and r based on bank angle/turn rate.
        # Presuming a steady state level turn, compute turn rate =
        # func(velocity, bank angle).  This is the one feed forward term used in
        # this set of control laws and it is purely physics based and works for
        # all fixed wing aircraft.
        phi_rad = att_node.getDouble("phi_deg") * d2r
        if abs(phi_rad) < pi * 0.5 * 0.9:
            turn_rate_rps = tan(phi_rad) * g / self.vtrue_filt_mps
        else:
            turn_rate_rps = 0
        # compute a baseline q and r for the presumed steady state level turn,
        # this is what we dampen towards
        baseline_q = sin(phi_rad) * turn_rate_rps
        baseline_r = cos(phi_rad) * turn_rate_rps
        if abs(phi_rad) < pi * 0.5 * 0.9:
            baseline_lf = 1 / cos(phi_rad)
        else:
            baseline_lf = 1.0
        # print("tr: %.3f" % turn_rate_rps, "q: %.3f %.3f" % (baseline_q, self.q), "r: %.3f %.3f" % (baseline_r, self.r))
        fcs_node.setDouble("baseline_q", baseline_q)
        fcs_node.setDouble("baseline_r", baseline_r)
        fcs_node.setDouble("baseline_lf", baseline_lf)

    def derive_envelope_limits(self, flaps_norm, qbar):
        # envelope protection (doesn't protect against a 'zoom' maneuver for
        # lack of a better name for it)
        max_az = inv_alpha_func(flaps_norm, self.alpha_limit_deg, qbar)
        max_lf = (max_az / g) * 0.9  # shave down the limit a bit

        if max_lf > 1.0:
            max_phi = acos(1 / max_lf)
        else:
            max_phi = 0
        max_bank_deg = max_phi * r2d
        if max_bank_deg < 5: max_bank_deg = 5  # always allow a little bit of bank

        fcs_node.setDouble("max_az", max_az)
        fcs_node.setDouble("max_load_factor", max_lf)
        fcs_node.setDouble("max_bank_deg", max_bank_deg)
        print("max bank:", max_bank_deg)

    def update(self, dt):
        flaps_norm = fcs_node.getDouble("posFlap_norm")
        qbar_filt = fcs_node.getDouble("qbar")

        # in the air vs on the ground?  (uses a sigmoid function between
        # threshold speeds)
        flying_confidence = self.is_flying.get_flying_confidence(self.vc_filt_mps)
        fcs_node.setDouble("flying_confidence", flying_confidence)

        # update state and filters
        self.compute_stuff()

        self.derive_envelope_limits(flaps_norm, qbar_filt)

        print("master switch:", inceptors_node.getBool("master_switch"))
        if inceptors_node.getBool("master_switch"):
            # the HIL network interface will relay/set the control_node values
            pass
        else:
            # pilot/joystick commands drive built in FBW control laws
            roll_rate_request = inceptors_node.getDouble("roll") * self.roll_stick_scale
            # pitch_rate_request = -inceptors_node.getDouble("pitch") * self.pitch_stick_scale
            load_factor_request = 1 + inceptors_node.getDouble("pitch") * self.lf_stick_scale
            yaw_rate_request = inceptors_node.getDouble("yaw") * self.yaw_stick_scale
            # beta_deg_request = -inceptors_node.getDouble("yaw") * self.yaw_stick_scale

            # flight control laws
            roll_cmd = self.fcs_lat.update(roll_rate_request, dt)
            # pitch_cmd = self.fcs_lon.update(pitch_rate_request)
            pitch_cmd = self.fcs_lon.update(load_factor_request, dt)
            yaw_cmd = self.fcs_yaw.update(yaw_rate_request)
            print("pids: %.2f %.2f" % (self.fcs_lat.pid.output, self.fcs_lon.pid.output))
            control_node.setDouble("aileron", roll_cmd)
            control_node.setDouble("rudder", yaw_cmd)
            # control_node.setDouble("rudder", 0)
            # control_node.setDouble("rudder", inceptors_node.getDouble("yaw"))
            control_node.setDouble("elevator", pitch_cmd)
            throttle_cmd = inceptors_node.getDouble("power")
            control_node.setDouble("throttle", throttle_cmd)

        # pass through flaps and throttle for now
        control_node.setBool("flaps_down", inceptors_node.getBool("flaps_down"))
        control_node.setBool("flaps_up", inceptors_node.getBool("flaps_up"))
