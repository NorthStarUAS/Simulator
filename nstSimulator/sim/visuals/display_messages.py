import struct
from PropertyTree import PropertyNode

# Message id constants
display_v1_id = 10
terrain_v2_id = 12

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

    def msg2props(self, node):
        node.setDouble("time_sec", self.time_sec)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("altitude_agl_m", self.altitude_agl_m)
        node.setDouble("vn_mps", self.vn_mps)
        node.setDouble("ve_mps", self.ve_mps)
        node.setDouble("vd_mps", self.vd_mps)
        node.setDouble("roll_deg", self.roll_deg)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("yaw_deg", self.yaw_deg)
        node.setDouble("gamma_deg", self.gamma_deg)
        node.setDouble("airspeed_kt", self.airspeed_kt)
        node.setDouble("ref_pressure_inhg", self.ref_pressure_inhg)
        node.setDouble("temp_C", self.temp_C)
        node.setDouble("ax_mps2", self.ax_mps2)
        node.setDouble("ay_mps2", self.ay_mps2)
        node.setDouble("az_mps2", self.az_mps2)
        node.setDouble("p_rps", self.p_rps)
        node.setDouble("q_rps", self.q_rps)
        node.setDouble("r_rps", self.r_rps)
        node.setDouble("ail_cmd_norm", self.ail_cmd_norm)
        node.setDouble("ele_cmd_norm", self.ele_cmd_norm)
        node.setDouble("rud_cmd_norm", self.rud_cmd_norm)
        node.setDouble("thr_cmd_norm", self.thr_cmd_norm)
        node.setDouble("flap_cmd_norm", self.flap_cmd_norm)
        node.setString("return_ip_addr", self.return_ip_addr)

    def props2msg(self, node):
        self.time_sec = node.getDouble("time_sec")
        self.longitude_deg = node.getDouble("longitude_deg")
        self.latitude_deg = node.getDouble("latitude_deg")
        self.altitude_m = node.getDouble("altitude_m")
        self.altitude_agl_m = node.getDouble("altitude_agl_m")
        self.vn_mps = node.getDouble("vn_mps")
        self.ve_mps = node.getDouble("ve_mps")
        self.vd_mps = node.getDouble("vd_mps")
        self.roll_deg = node.getDouble("roll_deg")
        self.pitch_deg = node.getDouble("pitch_deg")
        self.yaw_deg = node.getDouble("yaw_deg")
        self.gamma_deg = node.getDouble("gamma_deg")
        self.airspeed_kt = node.getDouble("airspeed_kt")
        self.ref_pressure_inhg = node.getDouble("ref_pressure_inhg")
        self.temp_C = node.getDouble("temp_C")
        self.ax_mps2 = node.getDouble("ax_mps2")
        self.ay_mps2 = node.getDouble("ay_mps2")
        self.az_mps2 = node.getDouble("az_mps2")
        self.p_rps = node.getDouble("p_rps")
        self.q_rps = node.getDouble("q_rps")
        self.r_rps = node.getDouble("r_rps")
        self.ail_cmd_norm = node.getDouble("ail_cmd_norm")
        self.ele_cmd_norm = node.getDouble("ele_cmd_norm")
        self.rud_cmd_norm = node.getDouble("rud_cmd_norm")
        self.thr_cmd_norm = node.getDouble("thr_cmd_norm")
        self.flap_cmd_norm = node.getDouble("flap_cmd_norm")
        self.return_ip_addr = node.getString("return_ip_addr")

# Message: terrain_v2
# Id: 12
class terrain_v2():
    id = 12
    _pack_string = "<ddf"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.longitude_deg = 0.0
        self.latitude_deg = 0.0
        self.terrain_height_m = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.longitude_deg,
                  self.latitude_deg,
                  self.terrain_height_m)
        return msg

    def unpack(self, msg):
        (self.longitude_deg,
         self.latitude_deg,
         self.terrain_height_m) = self._struct.unpack(msg)

    def msg2props(self, node):
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("terrain_height_m", self.terrain_height_m)

    def props2msg(self, node):
        self.longitude_deg = node.getDouble("longitude_deg")
        self.latitude_deg = node.getDouble("latitude_deg")
        self.terrain_height_m = node.getDouble("terrain_height_m")

