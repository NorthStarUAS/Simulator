# FlightGear network interface

import navpy
import socket
import struct

from nstSimulator.sim.lib.props import pos_node, vel_node, att_node, fcs_node

# KANE
# lat_deg = 45.13841697
# lon_deg = -93.2101002
# altitude_m = 400.0

# 64S
# lat_deg = 42.744
# lon_deg = -122.487
# altitude_m = 900

port = 6504
ip = "127.0.0.1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_to_fgfs():
    # lla = navpy.ned2lla(state.pos_ned, lat_deg, lon_deg, altitude_m)
    msg = struct.pack("!fddffffffffffffff",
                      0.0,
                      pos_node.getDouble("lat_geod_deg"), pos_node.getDouble("long_gc_deg"), pos_node.getDouble("geod_alt_m")/0.3048,
                      att_node.getDouble("phi_deg"),
                      att_node.getDouble("theta_deg"),
                      att_node.getDouble("psi_deg"),
                      vel_node.getDouble("vc_kts"),

                      fcs_node.getDouble("posAil_deg") / 12.5,
                      fcs_node.getDouble("posElev_deg") / 25,
                      -fcs_node.getDouble("posRud_deg") / 20,
                      fcs_node.getDouble("posThrottle_nd"),

                      -fcs_node.getDouble("posAil_deg") / 12.5, # Emannual's SR22 needs a negative value here, reversed from the other SR22
                      fcs_node.getDouble("posAil_deg") / 12.5,
                      fcs_node.getDouble("posElev_deg") / 25,
                      -fcs_node.getDouble("posRud_deg") / 20,
                      fcs_node.getDouble("posFlap_deg") / 32
                    )
    sock.sendto(msg, (ip, port))

