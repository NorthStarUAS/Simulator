import json
from math import atan2, sqrt
import numpy as np
import socket
import time

from direct.stdpy import threading

import navpy

from nstSimulator.sim.lib.props import att_node, pos_node, root_node, vel_node
from nstSimulator.utils.constants import r2d, m2ft
from .display_messages import display_v1, terrain_v2

port_in = 6767
port_out = 6768

comms_lock = threading.Lock()
comms_queue = []

class CommsWorker(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_in.bind( ("", port_in))

    def run(self):
        # start = time.time()
        # count = 0
        while True:
            data, addr = self.sock_in.recvfrom(1024*16)
            # count += 1
            # elapsed = time.time() - start
            # print("python sim rate:", count / elapsed)
            comms_lock.acquire()
            comms_queue.append(data)
            comms_lock.release()

class CommsManager():
    def __init__(self):
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.nedref = None
        self.nedref_time = -1
        self.lla = [0, 0, 0]  # order: lat_deg, lon_deg, alt_m
        self.nedpos = np.zeros(3)
        self.nedvel = np.zeros(3)
        self.hpr_deg = np.zeros(3)
        self.ail_cmd_norm = 0
        self.ele_cmd_norm = 0
        self.rud_cmd_norm = 0
        self.flap_cmd_norm = 0
        self.dt = None
        self.dlat = 0
        self.dlon = 0
        self.dalt = 0
        self.psiDot_dps_est = None
        self.return_ip_addr = None

        self.vc_kts_filt = 0.0
        self.alt_msl_filt = 0.0

        self.msg_prev = None

        self.comms_worker = CommsWorker()
        self.comms_worker.start()

    def angle_diff_deg(self, a1, a2):
        diff = a1 - a2
        if diff < -180: diff += 360
        if diff > 180: diff -= 360
        return diff

    def update(self):
        data = None
        comms_lock.acquire()
        if len(comms_queue):
            # if len(comms_queue) > 1:
                # print("comms queue:", len(comms_queue))
            data = comms_queue[-1]
            comms_queue.clear()
        comms_lock.release()

        if data is None:
            if self.dt is None or self.psiDot_dps_est is None:
                return
            else:
                # project ahead
                est_hz = 60
                self.lla[0] += self.dlat / est_hz
                self.lla[1] += self.dlon / est_hz
                self.lla[2] += self.dalt / est_hz
                self.hpr_deg[0] += self.psiDot_dps_est / est_hz
                self.hpr_deg[1] += self.thetaDot_dps_est / est_hz
                self.hpr_deg[2] += self.phiDot_dps_est / est_hz
        elif False:
            # accept new data
            msg = display_v1()
            msg.unpack(data)

            self.time_sec = msg.time_sec
            self.lla[0] = msg.latitude_deg
            self.lla[1] = msg.longitude_deg
            self.lla[2] = msg.altitude_m
            alt_ft = msg.altitude_m * m2ft
            self.hpr_deg[2] = msg.roll_deg
            self.hpr_deg[1] = msg.pitch_deg
            self.hpr_deg[0] = msg.yaw_deg
            self.indicated_kts = msg.airspeed_kt
            self.ail_cmd_norm = msg.ail_cmd_norm
            self.ele_cmd_norm = msg.ele_cmd_norm
            self.rud_cmd_norm = msg.rud_cmd_norm
            self.flap_cmd_norm = msg.flap_cmd_norm
            self.return_ip_addr = msg.return_ip_addr

            if self.msg_prev is not None:
                self.dt = msg.time_sec - self.msg_prev.time_sec
                if self.dt > 0:
                    self.dlat = (msg.latitude_deg - self.msg_prev.latitude_deg) / self.dt
                    self.dlon = (msg.longitude_deg - self.msg_prev.longitude_deg) / self.dt
                    self.dalt = (msg.altitude_m - self.msg_prev.altitude_m) / self.dt
                    self.psiDot_dps_est = self.angle_diff_deg(msg.yaw_deg, self.msg_prev.yaw_deg) / self.dt
                    self.thetaDot_dps_est = self.angle_diff_deg(msg.pitch_deg, self.msg_prev.pitch_deg) / self.dt
                    self.phiDot_dps_est = self.angle_diff_deg(msg.roll_deg, self.msg_prev.roll_deg) / self.dt
            self.msg_prev = msg
        else:
            root_node.set_json_string(data.decode())
            root_node.pretty_print()
            self.time_sec = root_node.getDouble("sim_time_sec")
            self.lla[0] = pos_node.getDouble("lat_geod_deg")
            self.lla[1] = pos_node.getDouble("long_gc_deg")
            self.lla[2] = pos_node.getDouble("geod_alt_m")
            self.hpr_deg[2] = att_node.getDouble("phi_deg")
            self.hpr_deg[1] = att_node.getDouble("theta_deg")
            self.hpr_deg[0] = att_node.getDouble("psi_deg")
            self.indicated_kts = vel_node.getDouble("vc_kts")
            # self.ail_cmd_norm = msg.ail_cmd_norm
            # self.ele_cmd_norm = msg.ele_cmd_norm
            # self.rud_cmd_norm = msg.rud_cmd_norm
            # self.flap_cmd_norm = msg.flap_cmd_norm
            self.return_ip_addr = root_node.getString("return_ip_addr")

            # if self.msg_prev is not None:
            #     self.dt = msg.time_sec - self.msg_prev.time_sec
            #     if self.dt > 0:
            #         self.dlat = (msg.latitude_deg - self.msg_prev.latitude_deg) / self.dt
            #         self.dlon = (msg.longitude_deg - self.msg_prev.longitude_deg) / self.dt
            #         self.dalt = (msg.altitude_m - self.msg_prev.altitude_m) / self.dt
            #         self.psiDot_dps_est = self.angle_diff_deg(msg.yaw_deg, self.msg_prev.yaw_deg) / self.dt
            #         self.thetaDot_dps_est = self.angle_diff_deg(msg.pitch_deg, self.msg_prev.pitch_deg) / self.dt
            #         self.phiDot_dps_est = self.angle_diff_deg(msg.roll_deg, self.msg_prev.roll_deg) / self.dt
            # self.msg_prev = msg

            # condition some values
            self.vc_kts_filt = 0.95 * self.vc_kts_filt + 0.05 * vel_node.getDouble("vc_kts")
            vel_node.setDouble("vc_kts_filt", self.vc_kts_filt)
            self.alt_msl_filt = 0.95 * self.alt_msl_filt + 0.05 * pos_node.getDouble("geod_alt_m") * m2ft
            pos_node.setDouble("alt_msl_filt", int(round(self.alt_msl_filt / 10))*10)

        # print("nedref:", self.nedref, "lla_deg/m:", self.lla)
        if self.nedref is None or np.linalg.norm(self.nedpos[:2]) > 1000:
            if self.lla is not None:
                self.nedref = [ self.lla[0], self.lla[1], 0.0 ]
                self.nedref_time = time.time()
                print("Updating nedref:", self.nedref, self.nedpos)
        if self.nedref is not None:
            # order: lat, lon, alt
            self.nedpos = navpy.lla2ned(self.lla[0], self.lla[1], self.lla[2], self.nedref[0], self.nedref[1], self.nedref[2])
            # print("nedpos:", self.nedpos)
            if np.linalg.norm(self.nedpos) > 100:
                nedref_need_update = True  # fixme never used?

        self.course_deg = 90 - atan2(self.nedvel[0], self.nedvel[1])*r2d
        # print("course_deg:", self.course_deg)
        #print(msg.ned_velocity)
        self.gs_mps = sqrt(self.nedvel[0]**2 + self.nedvel[1]**2)

    def send(self, ground_elev_m):
        if self.return_ip_addr is not None:
            # reply with terrain height
            msg_out = terrain_v2()
            msg_out.latitude_deg = self.lla[0]
            msg_out.longitude_deg = self.lla[1]
            msg_out.terrain_height_m = ground_elev_m
            self.sock_out.sendto(msg_out.pack(), (self.return_ip_addr, port_out))
            # print("(send) sending: %.1f" % ground_elev_m)

    def get_ned_from_lla(self, lat, lon, alt):
        if self.nedref is not None:
            return navpy.lla2ned(lat, lon, alt, self.nedref[0], self.nedref[1], self.nedref[2])
        else:
            return [0, 0, 0]

