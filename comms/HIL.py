import socket

from lib.props import airdata_node, gps_node, imu_node

from .ns_messages import airdata_v8, gps_v5, imu_v6
from .serial_parser import wrap_packet

link_host = "localhost"
link_port = 5051

class HIL():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def write(self):
        msg = airdata_v8()
        msg.props2msg(airdata_node)
        msg.index = 0
        buf = msg.pack()
        packet = wrap_packet(msg.id, buf)
        self.sock_out.sendto(packet, (link_host, link_port))

        msg = imu_v6()
        msg.props2msg(imu_node)
        msg.index = 0
        buf = msg.pack()
        packet = wrap_packet(msg.id, buf)
        self.sock_out.sendto(packet, (link_host, link_port))

        msg = gps_v5()
        msg.props2msg(gps_node)
        msg.index = 0
        buf = msg.pack()
        packet = wrap_packet(msg.id, buf)
        self.sock_out.sendto(packet, (link_host, link_port))

    def read(self):
        pkt_id = self.parser.read(self.ser)
        if pkt_id >= 0:
            print("received:", pkt_id)
            # parse_msg(pkt_id, parser.payload)
            # log_msg(f, pkt_id, parser.pkt_len, parser.payload,
                    # parser.cksum_lo, parser.cksum_hi)
