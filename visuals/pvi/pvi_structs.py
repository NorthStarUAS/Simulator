import struct

# Message id constants
sensors_v2_id = 12
sensors_v3_id = 15
effectors_v2_id = 13
visual_state_v1_id = 14

# Message: sensors_v2
# Id: 12
class sensors_v2():
    id = 12
    _pack_string = "<dddfffffffffffffffff"
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
        self.airspeed_kt = 0.0
        self.ref_pressure_inhg = 0.0
        self.temp_C = 0.0
        self.ax_mps2 = 0.0
        self.ay_mps2 = 0.0
        self.az_mps2 = 0.0
        self.p_rps = 0.0
        self.q_rps = 0.0
        self.r_rps = 0.0
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
                  self.airspeed_kt,
                  self.ref_pressure_inhg,
                  self.temp_C,
                  self.ax_mps2,
                  self.ay_mps2,
                  self.az_mps2,
                  self.p_rps,
                  self.q_rps,
                  self.r_rps)
        return msg

    def unpack(self, msg):
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
         self.airspeed_kt,
         self.ref_pressure_inhg,
         self.temp_C,
         self.ax_mps2,
         self.ay_mps2,
         self.az_mps2,
         self.p_rps,
         self.q_rps,
         self.r_rps) = self._struct.unpack(msg)
# Message: sensors_v3
# Id: 15
class sensors_v3():
    id = 15
    _pack_string = "<dddfffffffffffffffffffff"
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
        self.airspeed_kt = 0.0
        self.ref_pressure_inhg = 0.0
        self.temp_C = 0.0
        self.ax_mps2 = 0.0
        self.ay_mps2 = 0.0
        self.az_mps2 = 0.0
        self.p_rps = 0.0
        self.q_rps = 0.0
        self.r_rps = 0.0
        self.psiDot_dps = 0.0
        self.hDot_mps = 0.0
        self.refPhi_deg = 0.0
        self.refTheta_deg = 0.0
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
                  self.airspeed_kt,
                  self.ref_pressure_inhg,
                  self.temp_C,
                  self.ax_mps2,
                  self.ay_mps2,
                  self.az_mps2,
                  self.p_rps,
                  self.q_rps,
                  self.r_rps,
                  self.psiDot_dps,
                  self.hDot_mps,
                  self.refPhi_deg,
                  self.refTheta_deg)
        return msg

    def unpack(self, msg):
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
         self.airspeed_kt,
         self.ref_pressure_inhg,
         self.temp_C,
         self.ax_mps2,
         self.ay_mps2,
         self.az_mps2,
         self.p_rps,
         self.q_rps,
         self.r_rps,
         self.psiDot_dps,
         self.hDot_mps,
         self.refPhi_deg,
         self.refTheta_deg) = self._struct.unpack(msg)
# Message: effectors_v2
# Id: 13
class effectors_v2():
    id = 13
    _pack_string = "<dfffff"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.time_sec = 0.0
        self.aileron_norm = 0.0
        self.elevator_norm = 0.0
        self.throttle_norm = 0.0
        self.rudder_norm = 0.0
        self.flaps_norm = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.time_sec,
                  self.aileron_norm,
                  self.elevator_norm,
                  self.throttle_norm,
                  self.rudder_norm,
                  self.flaps_norm)
        return msg

    def unpack(self, msg):
        (self.time_sec,
         self.aileron_norm,
         self.elevator_norm,
         self.throttle_norm,
         self.rudder_norm,
         self.flaps_norm) = self._struct.unpack(msg)
# Message: visual_state_v1
# Id: 14
class visual_state_v1():
    id = 14
    _pack_string = "<dddfffffffff"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.time_sec = 0.0
        self.longitude_deg = 0.0
        self.latitude_deg = 0.0
        self.altitude_m = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.aileron_norm = 0.0
        self.elevator_norm = 0.0
        self.throttle_norm = 0.0
        self.rudder_norm = 0.0
        self.flaps_norm = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.time_sec,
                  self.longitude_deg,
                  self.latitude_deg,
                  self.altitude_m,
                  self.roll_deg,
                  self.pitch_deg,
                  self.yaw_deg,
                  self.aileron_norm,
                  self.elevator_norm,
                  self.throttle_norm,
                  self.rudder_norm,
                  self.flaps_norm)
        return msg

    def unpack(self, msg):
        (self.time_sec,
         self.longitude_deg,
         self.latitude_deg,
         self.altitude_m,
         self.roll_deg,
         self.pitch_deg,
         self.yaw_deg,
         self.aileron_norm,
         self.elevator_norm,
         self.throttle_norm,
         self.rudder_norm,
         self.flaps_norm) = self._struct.unpack(msg)
