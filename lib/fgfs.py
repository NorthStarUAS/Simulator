# FlightGear network interface

import navpy
import socket
import struct

from lib.constants import mps2kt, r2d

# KANE
lat_deg = 45.13841697
lon_deg = -93.2101002
altitude_m = 400.0

#64S
lat_deg = 42.744
lon_deg = -122.487
altitude_m = 900

port = 6504
ip = "127.0.0.1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_to_fgfs(sim):
    lla = navpy.ned2lla(sim.pos_ned, lat_deg, lon_deg, altitude_m)
    msg = struct.pack("!ddfffff",
                      lla[0], lla[1], lla[2]/0.3048,
                      sim.state_mgr.phi_rad*r2d,
                      sim.state_mgr.the_rad*r2d,
                      sim.state_mgr.psi_rad*r2d,
                      sim.airspeed_mps*mps2kt)
    sock.sendto(msg, (ip, port))
    
