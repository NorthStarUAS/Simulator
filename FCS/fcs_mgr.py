from lib.props import control_engine_node, control_flight_node, inceptor_node, vel_node

from FCS.direct_pbeta import pbeta_controller
from FCS.direct_q import q_controller
from FCS.util import IsFlying

class FCSMgr():
    def __init__(self):
        self.fcs_lat = pbeta_controller()
        self.fcs_lon = q_controller()
        self.is_flying = IsFlying(on_ground_for_sure_mps=30, flying_for_sure_mps=40)

    def update(self):
        flying_confidence = self.is_flying.get_flying_confidence(vel_node.getFloat("vc_mps"))

        self.fcs_lat.update(flying_confidence)
        self.fcs_lon.update(flying_confidence)
        print("integrators: %.2f %.2f %.2f" % (self.fcs_lat.aileron_int, self.fcs_lon.elevator_int, self.fcs_lat.rudder_int))

        control_flight_node.setFloat("aileron", self.fcs_lat.aileron_cmd)
        control_flight_node.setFloat("rudder", self.fcs_lat.rudder_cmd)
        control_flight_node.setFloat("elevator", self.fcs_lon.elevator_cmd)

        # pass through flaps and throttle for now
        control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
        control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))
        throttle_cmd = inceptor_node.getFloat("throttle")
        control_engine_node.setFloat("throttle", throttle_cmd)
