import socket

from PropertyTree import PropertyNode

from nstSimulator.sim.lib.props import att_node, pos_node, root_node, vel_node
from .display_messages import display_v1, terrain_v2

display_host = "localhost"
display_port_in = 6768
display_port_out = 6767

class Display():
    def __init__(self):
        # outgoing
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # incoming
        hostname = socket.gethostname()
        self.ip_addr = socket.gethostbyname(hostname)
        print(hostname, self.ip_addr)
        root_node.setString("return_ip_addr", self.ip_addr)
        self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_in.bind( ("", display_port_in))
        self.sock_in.setblocking(False)

    def send_msg(self):
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
        msg_out.return_ip_addr = self.ip_addr
        # print(msg_out.__dict__)
        self.sock_out.sendto(msg_out.pack(), (display_host, display_port_out))

    def send(self):
        msg = PropertyNode("/").get_json_string() + '\r\n'
        self.sock_out.sendto(str.encode(msg), (display_host, display_port_out))

    def receive(self):
        try:
            while True:
                # burn through all pending packets so we don't get behind
                data, addr = self.sock_in.recvfrom(1024)
                if data is not None:
                    values = terrain_v2()
                    values.unpack(data)
                    diff_lat = abs(pos_node.getDouble("lat_geod_deg") - values.latitude_deg)
                    diff_lon = abs(pos_node.getDouble("long_gc_deg") - values.longitude_deg)
                    if diff_lat < 0.01 and diff_lon < 0.01:
                        # print("set terrain elevation from visual system: %.1f" % self.terrain_height_m)
                        pos_node.setDouble("visual_terrain_elevation_m", values.terrain_height_m)
                    else:
                        print("ignoring elevation from previous position")
        except:
            pass # no more pending data

    def update(self):
        self.send()
        self.receive()

    def __del__(self):
        self.sock_in.close()
        self.sock_out.close()
        print("deleting self: svo")
