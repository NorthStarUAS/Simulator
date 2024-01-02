from math import atan2, sqrt
import numpy as np
import socket
import struct
import time

from direct.stdpy import threading

import navpy

from world.constants import ft2m, r2d

port_in = 6504

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
            data, addr = self.sock_in.recvfrom(1024)
            # count += 1
            # elapsed = time.time() - start
            # print("python sim rate:", count / elapsed)
            comms_lock.acquire()
            comms_queue.append(data)
            comms_lock.release()

class CommsManager():
    def __init__(self):
        self.fgfs_pack_string = "!fddffffffffff"
        self.fgfs_struct = struct.Struct(self.fgfs_pack_string)

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
        self.values_prev = None
        self.psiDot_dps_est = None

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
        elif len(data) == 60:
            # accept new data from flightgear rc-sim.xml data packet format
            values = self.fgfs_struct.unpack(data)
            (self.time_sec,
            self.lla[0],
            self.lla[1],
            alt_ft,
            self.hpr_deg[2],
            self.hpr_deg[1],
            self.hpr_deg[0],
            self.indicated_kts,
            right_ail,
            left_ail,
            self.ele_cmd_norm,
            self.rud_cmd_norm,
            self.flap_cmd_norm,
            ) = values
            self.lla[2] = alt_ft * ft2m
            self.ail_cmd_norm = (right_ail + left_ail) * 0.5

            if self.values_prev is not None:
                self.dt = values[0] - self.values_prev[0]
                if self.dt > 0:
                    self.dlat = (values[1] - self.values_prev[1]) / self.dt
                    self.dlon = (values[2] - self.values_prev[2]) / self.dt
                    self.dalt = (values[3] - self.values_prev[3]) / self.dt
                    self.psiDot_dps_est = self.angle_diff_deg(values[6], self.values_prev[6]) / self.dt
                    self.thetaDot_dps_est = self.angle_diff_deg(values[5], self.values_prev[5]) / self.dt
                    self.phiDot_dps_est = self.angle_diff_deg(values[4], self.values_prev[4]) / self.dt
            self.values_prev = values
        else:
            print("wrong length:", len(data))
            return

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

    def get_ned_from_lla(self, lat, lon, alt):
        if self.nedref is not None:
            return navpy.lla2ned(lat, lon, alt, self.nedref[0], self.nedref[1], self.nedref[2])
        else:
            return [0, 0, 0]

