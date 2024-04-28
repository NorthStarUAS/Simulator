from lib.constants import d2r
from lib.props import control_engine_node, control_flight_node, inceptor_node, vel_node

from FCS.direct_pbeta import pbeta_controller
from FCS.direct_q import q_controller
from FCS.util import IsFlying

class FCSMgr():
    def __init__(self):
        # stick -> rate command scaling
        self.roll_stick_scale = 30 * d2r   # rad
        self.pitch_stick_scale = 30 * d2r  # rad
        self.yaw_stick_scale = 20          # maps to beta_deg

        self.fcs_lat = pbeta_controller()
        self.fcs_lon = q_controller()
        self.is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=40)

    def update(self):
        # Pilot commands
        roll_rate_request = inceptor_node.getFloat("aileron") * self.roll_stick_scale
        pitch_rate_request = -inceptor_node.getFloat("elevator") * self.pitch_stick_scale
        beta_deg_request = -inceptor_node.getFloat("rudder") * self.yaw_stick_scale

        flying_confidence = self.is_flying.get_flying_confidence(vel_node.getFloat("vc_mps"))

        roll_cmd, yaw_cmd = self.fcs_lat.update(roll_rate_request, beta_deg_request, flying_confidence)
        pitch_cmd = self.fcs_lon.update(pitch_rate_request, flying_confidence)
        print("integrators: %.2f %.2f %.2f" % (self.fcs_lat.roll_int, self.fcs_lon.pitch_int, self.fcs_lat.yaw_int))
        control_flight_node.setFloat("aileron", roll_cmd)
        control_flight_node.setFloat("rudder", yaw_cmd)
        control_flight_node.setFloat("elevator", pitch_cmd)

        # pass through flaps and throttle for now
        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))
        throttle_cmd = inceptor_node.getFloat("throttle")
        control_engine_node.setFloat("throttle", throttle_cmd)
