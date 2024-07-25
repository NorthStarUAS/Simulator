from scipy.interpolate import interp1d
import socket

from lib.props import airdata_node, fcs_node, gps_node, imu_node, inceptor_node, power_node

from .ns_messages import airdata_v8, gps_v5, imu_v6, inceptors_v1, power_v1
from .serial_parser import wrap_packet

link_host = "localhost"
link_port = 5051

class HIL():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_gps_millis = 0.0

        self.batt_ah = 10
        self.max_amp = 15
        self.batt_last_time = 0
        self.batt_used_ah = 0

    def fake_battery(self):
        cells = 4
        batv = [ 3.3, 3.50, 3.65, 3.80, 4.20 ]
        batp = [ 0.0, 0.05, 0.27, 0.83, 1.00 ]
        batf = interp1d(batp, batv)

        imu_millis = imu_node.getUInt("millis")
        imu_time = imu_millis / 1000
        dt = imu_time - self.batt_last_time
        self.batt_last_time = imu_time
        if dt > 0.1:
            dt = 0.1
        thr = fcs_node.getDouble("cmdThrottle_nd")
        dah = thr*self.max_amp * dt / 3600
        self.batt_used_ah += dah
        batt_perc = (self.batt_ah - self.batt_used_ah) / self.batt_ah
        if batt_perc < 0:
            # this is a totally fake battery model intended only to generate
            # some plausible numbers for testing, so recycle the battery to 100%
            # when it's empty and just keep going.
            self.batt_used_ah = 0
        sag = thr * 0.1
        volt = (batf(batt_perc) - sag) * cells
        print("dt: %.2f" % dt, "batt: %.3f" % batt_perc, "dah:", dah, "volt: %.2f" % volt)
        power_node.setUInt("millis", imu_millis)
        power_node.setDouble("avionics_vcc", 5 - sag*0.1)
        power_node.setDouble("main_vcc", volt)
        power_node.setDouble("cell_vcc", volt/cells)
        power_node.setDouble("main_amps", thr*self.max_amp)
        power_node.setDouble("total_mah", self.batt_used_ah*1000)

    def write(self):
        self.fake_battery()

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

        msg = inceptors_v1()
        msg.index = 0
        msg.millis = imu_node.getUInt("millis")
        msg.channel[0] = inceptor_node.getDouble("throttle")
        msg.channel[1] = inceptor_node.getDouble("aileron")
        msg.channel[2] = inceptor_node.getDouble("elevator")
        msg.channel[3] = inceptor_node.getDouble("rudder")
        msg.channel[4] = fcs_node.getDouble("cmdFlap_deg")
        msg.channel[5] = inceptor_node.getDouble("gear")
        buf = msg.pack()
        packet = wrap_packet(msg.id, buf)
        self.sock_out.sendto(packet, (link_host, link_port))
        # print("sending inceptors:", msg.id, msg.__dict__)

        msg = gps_v5()
        msg.props2msg(gps_node)
        if msg.millis >= self.last_gps_millis + 200:
            # 5 hz gps
            self.last_gps_millis += 200
            msg.index = 0
            buf = msg.pack()
            packet = wrap_packet(msg.id, buf)
            self.sock_out.sendto(packet, (link_host, link_port))

        msg = power_v1()
        msg.props2msg(power_node)
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
