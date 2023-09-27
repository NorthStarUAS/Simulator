from math import pi
import socket

from visuals.pvi.pvi_structs import sensors_v3

pvi_host = "localhost"
pvi_port = 6767

r2d = 180.0 / pi
ft2m = 0.3048

class PVI():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self, sim, psiDot_dps, hDot_mps, refPhi_deg, refTheta_deg):
        # ail_norm = sim.fdm['fcs/right-aileron-pos-norm']
        # ele_norm = sim.fdm['fcs/elevator-pos-norm']
        # rud_norm = sim.fdm['fcs/rudder-pos-norm']

        msg_out = sensors_v3()
        # msg_out.time_sec = dd.simdata["Local Time"]
        msg_out.longitude_deg = sim.fdm['position/long-gc-rad'] * r2d
        msg_out.latitude_deg = sim.fdm['position/lat-geod-rad'] * r2d
        msg_out.altitude_m = sim.fdm['position/geod-alt-ft'] * ft2m
        # msg_out.altitude_agl_m = dd.simdata["Plane Alt Above Ground"] * ft2m
        msg_out.vn_mps = sim.fdm["velocities/v-north-fps"] * ft2m
        msg_out.ve_mps = sim.fdm["velocities/v-east-fps"] * ft2m
        msg_out.vd_mps = sim.fdm["velocities/v-down-fps"] * ft2m
        msg_out.roll_deg = sim.fdm['attitude/phi-rad'] * r2d
        msg_out.pitch_deg = sim.fdm['attitude/theta-rad'] * r2d
        msg_out.yaw_deg = sim.fdm['attitude/psi-rad'] * r2d
        # msg_out.p_rps = -dd.simdata["Rotation Velocity Body Z"] # fixme: need to validate units / coordinate system alignment
        # msg_out.q_rps = -dd.simdata["Rotation Velocity Body X"]
        # msg_out.r_rps = -dd.simdata["Rotation Velocity Body Y"]
        # msg_out.ax_mps2 = -dd.simdata["Acceleration Body Z"] * ft2m  # fixme: doesn't include "g" (also not in ned frame of reference)
        # msg_out.ay_mps2 = -dd.simdata["Acceleration Body X"] * ft2m
        # msg_out.az_mps2 = -dd.simdata["Acceleration Body Y"] * ft2m
        msg_out.airspeed_kt = sim.fdm['velocities/vtrue-kts']
        # msg_out.temp_C = dd.simdata["Total Air Temperature"]
        # msg_out.ref_pressure_inhg = dd.simdata["Barometer Pressure"] # fixme: validate this is the variable we want

        msg_out.psiDot_dps = psiDot_dps
        msg_out.hDot_mps = hDot_mps
        msg_out.refPhi_deg = refPhi_deg
        msg_out.refTheta_deg = refTheta_deg

        # print(msg_out.__dict__)
        self.sock_out.sendto(msg_out.pack(), (pvi_host, pvi_port))