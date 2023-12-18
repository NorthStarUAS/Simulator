from math import pi
import socket

import navpy

from lib.constants import mps2kt, r2d
from visuals.pvi.pvi_structs import sensors_v3

# KANE
# lat_deg = 45.13841697
# lon_deg = -93.2101002
# altitude_m = 400.0

#64S
lat_deg = 42.744
lon_deg = -122.487
altitude_m = 900

pvi_host = "localhost"
pvi_port = 6767

ft2m = 0.3048

class PVI():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self, state, psiDot_dps, hDot_mps, refPhi_deg, refTheta_deg):
        # ail_norm = sim.fdm['fcs/right-aileron-pos-norm']
        # ele_norm = sim.fdm['fcs/elevator-pos-norm']
        # rud_norm = sim.fdm['fcs/rudder-pos-norm']

        lla = navpy.ned2lla(state.pos_ned, lat_deg, lon_deg, altitude_m)

        msg_out = sensors_v3()
        # msg_out.time_sec = dd.simdata["Local Time"]
        msg_out.longitude_deg = lla[1]
        msg_out.latitude_deg = lla[0]
        msg_out.altitude_m = lla[2]
        # msg_out.altitude_agl_m = dd.simdata["Plane Alt Above Ground"] * ft2m
        msg_out.vn_mps = state.vel_ned[0]
        msg_out.ve_mps = state.vel_ned[1]
        msg_out.vd_mps = state.vel_ned[2]
        msg_out.roll_deg = state.phi_rad * r2d
        msg_out.pitch_deg = state.the_rad * r2d
        msg_out.yaw_deg = state.psi_rad * r2d
        # msg_out.p_rps = -dd.simdata["Rotation Velocity Body Z"] # fixme: need to validate units / coordinate system alignment
        # msg_out.q_rps = -dd.simdata["Rotation Velocity Body X"]
        # msg_out.r_rps = -dd.simdata["Rotation Velocity Body Y"]
        # msg_out.ax_mps2 = -dd.simdata["Acceleration Body Z"] * ft2m  # fixme: doesn't include "g" (also not in ned frame of reference)
        # msg_out.ay_mps2 = -dd.simdata["Acceleration Body X"] * ft2m
        # msg_out.az_mps2 = -dd.simdata["Acceleration Body Y"] * ft2m
        msg_out.airspeed_kt = state.airspeed_mps*mps2kt
        # msg_out.temp_C = dd.simdata["Total Air Temperature"]
        # msg_out.ref_pressure_inhg = dd.simdata["Barometer Pressure"] # fixme: validate this is the variable we want

        msg_out.psiDot_dps = psiDot_dps
        msg_out.hDot_mps = hDot_mps
        msg_out.refPhi_deg = refPhi_deg
        msg_out.refTheta_deg = refTheta_deg

        # print(msg_out.__dict__)
        self.sock_out.sendto(msg_out.pack(), (pvi_host, pvi_port))