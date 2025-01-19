import struct

# Message id constants
display_v1_id = 10
terrain_v1_id = 11

# Message: display_v1
# Id: 10
class display_v1():
    id = 10
    _pack_string = "<dddfffffffffffffffffffffffH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.time_sec = 0.0
        self.longitude_deg = 0.0
        self.latitude_deg = 0.0
        self.altitude_m = 0.0
        self.altitude_agl_m = 0.0
        self.vn_mps = 0.0
        self.ve_mps = 0.0
        self.vd_mps = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.gamma_deg = 0.0
        self.airspeed_kt = 0.0
        self.ref_pressure_inhg = 0.0
        self.temp_C = 0.0
        self.ax_mps2 = 0.0
        self.ay_mps2 = 0.0
        self.az_mps2 = 0.0
        self.p_rps = 0.0
        self.q_rps = 0.0
        self.r_rps = 0.0
        self.ail_cmd_norm = 0.0
        self.ele_cmd_norm = 0.0
        self.rud_cmd_norm = 0.0
        self.thr_cmd_norm = 0.0
        self.flap_cmd_norm = 0.0
        self.return_ip_addr = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.time_sec,
                  self.longitude_deg,
                  self.latitude_deg,
                  self.altitude_m,
                  self.altitude_agl_m,
                  self.vn_mps,
                  self.ve_mps,
                  self.vd_mps,
                  self.roll_deg,
                  self.pitch_deg,
                  self.yaw_deg,
                  self.gamma_deg,
                  self.airspeed_kt,
                  self.ref_pressure_inhg,
                  self.temp_C,
                  self.ax_mps2,
                  self.ay_mps2,
                  self.az_mps2,
                  self.p_rps,
                  self.q_rps,
                  self.r_rps,
                  self.ail_cmd_norm,
                  self.ele_cmd_norm,
                  self.rud_cmd_norm,
                  self.thr_cmd_norm,
                  self.flap_cmd_norm,
                  len(self.return_ip_addr))
        msg += str.encode(self.return_ip_addr)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.time_sec,
         self.longitude_deg,
         self.latitude_deg,
         self.altitude_m,
         self.altitude_agl_m,
         self.vn_mps,
         self.ve_mps,
         self.vd_mps,
         self.roll_deg,
         self.pitch_deg,
         self.yaw_deg,
         self.gamma_deg,
         self.airspeed_kt,
         self.ref_pressure_inhg,
         self.temp_C,
         self.ax_mps2,
         self.ay_mps2,
         self.az_mps2,
         self.p_rps,
         self.q_rps,
         self.r_rps,
         self.ail_cmd_norm,
         self.ele_cmd_norm,
         self.rud_cmd_norm,
         self.thr_cmd_norm,
         self.flap_cmd_norm,
         self.return_ip_addr_len) = self._struct.unpack(msg)
        self.return_ip_addr = extra[:self.return_ip_addr_len].decode()
        extra = extra[self.return_ip_addr_len:]
# Message: terrain_v1
# Id: 11
class terrain_v1():
    id = 11
    _pack_string = "<f"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.terrain_height_m = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.terrain_height_m)
        return msg

    def unpack(self, msg):
        (self.terrain_height_m,) = self._struct.unpack(msg)
