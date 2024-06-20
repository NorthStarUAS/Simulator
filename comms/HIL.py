from serial import Serial

from lib.props import gps_node, imu_node

from .ns_messages import gps_v5, imu_v6
from . import serial_parser

Serial("/dev/ttyACM0", 500000)

class HIL():
    def __init__(self):
        port = "/dev/ttyACM0"
        self.ser = Serial(port, 500000)
        self.parser = serial_parser.serial_parser()

        try:
            self.ser = Serial(port, 500000)
        except:
            print("Cannot open:", port)
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            print("Available ports:")
            for p in ports:
                print(p)
            quit()

    def write(self):
        msg = imu_v6()
        msg.props2msg(imu_node)
        msg.index = 0   # gps 0
        buf = msg.pack()
        packet = serial_parser.wrap_packet(msg.id, buf)
        result = self.ser.write(packet)
        if result != len(packet):
            print("ERROR: wrote %d of %d bytes to serial port!\n" % (result, len(packet)))

        msg = gps_v5()
        msg.props2msg(gps_node)
        msg.index = 0   # imu 0
        buf = msg.pack()
        packet = serial_parser.wrap_packet(msg.id, buf)
        result = self.ser.write(packet)
        if result != len(packet):
            print("ERROR: wrote %d of %d bytes to serial port!\n" % (result, len(packet)))


        return result

    def read(self):
        pkt_id = self.parser.read(self.ser)
        if pkt_id >= 0:
            print("received:", pkt_id)
            # parse_msg(pkt_id, parser.payload)
            # log_msg(f, pkt_id, parser.pkt_len, parser.payload,
                    # parser.cksum_lo, parser.cksum_hi)
