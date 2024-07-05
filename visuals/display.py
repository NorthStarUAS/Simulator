import socket

import navpy

from lib.props import att_node, pos_node, vel_node
from visuals.display_messages import display_v1

if False:
    # KANE
    # lat_deg = 45.13841697
    # lon_deg = -93.2101002
    # altitude_m = 400.0

    #64S
    lat_deg = 42.744
    lon_deg = -122.487
    altitude_m = 900
    lla = navpy.ned2lla(state.pos_ned, lat_deg, lon_deg, altitude_m)

display_host = "localhost"
display_port = 6767

class Display():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self):
        msg_out = display_v1()
        msg_out.longitude_deg = pos_node.getDouble("long_gc_deg")
        msg_out.latitude_deg = pos_node.getDouble("lat_geod_deg")
        msg_out.altitude_m = pos_node.getDouble("geod_alt_m")
        print("msg_out.altitude_m:", msg_out.altitude_m)
        msg_out.vn_mps = vel_node.getDouble("vn_mps")
        msg_out.ve_mps = vel_node.getDouble("ve_mps")
        msg_out.vd_mps = vel_node.getDouble("vd_mps")
        msg_out.roll_deg = att_node.getDouble("phi_deg")
        msg_out.pitch_deg = att_node.getDouble("theta_deg")
        msg_out.yaw_deg = att_node.getDouble("psi_deg")
        msg_out.airspeed_kt = vel_node.getDouble("vc_kts")
        # print(msg_out.__dict__)

        self.sock_out.sendto(msg_out.pack(), (display_host, display_port))