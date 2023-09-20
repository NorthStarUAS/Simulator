from math import pi

import navpy
from lib.constants import mps2kt, r2d

from .XPlaneUdp import *

# units
ft2m = 0.3048
m2ft = 1.0 / ft2m


class XPlane():
    def __init__(self):
        # KANE
        self.lat_deg = 45.13841697
        self.lon_deg = -93.2101002
        self.altitude_m = 400.0

        #64S
        self.lat_deg = 42.744
        self.lon_deg = -122.487
        self.altitude_m = 900

        self.latest_recv = 0
        self.last_sim_time = 0
        self.settle = None

        self.xp = XPlaneUdp()
        self.xp_ip = None
        self.xp_port = None
        self.sock = None

        try:
            beacon = self.xp.FindIp()
            print(beacon)
            if "IP" in beacon:
                self.xp_ip = beacon["IP"]
                self.xp_port = beacon["Port"]
        except XPlaneVersionNotSupported:
            print("XPlane Version not supported.")

        if self.xp_ip is None:
            print("No running X-Plane found, sorry.")
        else:
            print("XPlane found at:", self.xp_ip, self.xp_port)

        if self.xp_ip is not None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self, sim):
        self.send(sim)
        # self.receive(sim)

    def send_data_ref(self, name, value):
        msg = struct.pack('<4sxf500s', b'DREF',
                          value,
                          name.encode('utf-8'))
        self.sock.sendto(msg, (self.xp_ip, self.xp_port))

    def send(self, sim):
        if self.sock is not None:
            lla = navpy.ned2lla(sim.pos_ned, self.lat_deg, self.lon_deg, self.altitude_m)
            # Drive the external MSFS visual system
            lat_deg = lla[0]
            lon_deg = lla[1]
            alt_ft = lla[2] * m2ft
            alt_m = lla[2]
            phi_deg = sim.state_mgr.phi_rad * r2d
            the_deg = sim.state_mgr.the_rad * r2d
            psi_deg = sim.state_mgr.psi_rad * r2d
            # vtrue = sim.state_mgr.airspeed_mps * mps2kt
            vc =sim.state_mgr.airspeed_mps * mps2kt
            vd = sim.state_mgr.v_ned[2] *m2ft
            # ail_norm = sim.fdm['fcs/right-aileron-pos-norm']
            # https://www.siminnovations.com/xplane/dataref/?name=sim%2Fcockpit&type=float&writable=y&units=&description=&submit=Search
            # self.send_data_ref("sim/aircraft/parts/acf_elev", sim.fdm['fcs/left-aileron-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab1_elv1def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab1_elv2def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab2_elv1def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab2_elv2def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/joystick/yoke_pitch_ratio", sim.fdm['fcs/elevator-pos-norm'])
            # ele_norm = sim.fdm['fcs/elevator-pos-norm']
            # rud_norm = sim.fdm['fcs/rudder-pos-norm']
            #print(lat_rad, lon_rad, alt_ft, phi, the, psi)
            self.send_data_ref("sim/cockpit/gyros/phi_ind_ahars_pilot_deg", sim.state_mgr.phi_rad * r2d)
            self.send_data_ref("sim/cockpit/gyros/the_ind_ahars_pilot_deg", sim.state_mgr.the_rad * r2d)
            self.send_data_ref("sim/cockpit/gyros/psi_ind_ahars_pilot_degm", sim.state_mgr.psi_rad * r2d) # fixme: sending true, but calling it mag
            self.send_data_ref("sim/flightmodel/position/indicated_airspeed", vc)
            self.send_data_ref("sim/flightmodel/misc/h_ind", alt_ft)
            self.send_data_ref("sim/flightmodel/position/vh_ind_fpm", -vd*60)
            msg = struct.pack('<4sxidddfff', b'VEHX',
                  0,              # The index of the airplane you want to control.
                  lat_deg,        # latitude, in degrees
                  lon_deg,        # longitude, in degrees
                  alt_m,          # elevation above sea level, in meters
                  psi_deg,        # heading, degrees true
                  the_deg,        # pitch, degrees
                  phi_deg)        # roll, degrees
            self.sock.sendto(msg, (self.xp_ip, self.xp_port))

    def receive(self, sim):
        if self.sc is not None:
            while self.sc.receive(timeout_seconds=0.001):
                pass  # catch up the queue
            n = len(self.dd.simdata.changedsince(self.latest_recv))
            if n:
                # new state update received from sim
                self.latest_recv = self.dd.simdata.latest()
                time_sec = self.dd.simdata["Local Time"]
                agl_ft = self.dd.simdata["Plane Alt Above Ground"]
                alt_ft = sim.fdm['position/geod-alt-ft']
                ground_elev_ft = alt_ft - (agl_ft + 0.25)
                print("AGL (ft):", agl_ft, "Ground (ft):", ground_elev_ft)
                if self.settle is None:
                    self.settle = time_sec + 1
                if time_sec > self.settle:
                    sim.fdm["position/terrain-elevation-asl-ft"] = ground_elev_ft

                # pos_node["longitude_deg"] = self.dd.simdata["Plane Longitude"] * r2d
                # pos_node["latitude_deg"] = self.dd.simdata["Plane Latitude"] * r2d
                # pos_node["altitude_m"] = self.dd.simdata["Plane Altitude"] * ft2m
                # pos_node["altitude_agl_m"] = self.dd.simdata["Plane Alt Above Ground"] * ft2m
                # vel_node["vn_mps"] = self.dd.simdata["Velocity World Z"] * ft2m
                # vel_node["ve_mps"] = self.dd.simdata["Velocity World X"] * ft2m
                # vel_node["vd_mps"] = -self.dd.simdata["Velocity World Y"] * ft2m
                # orient_node["roll_deg"] = -self.dd.simdata["Plane Bank Degrees"] * r2d
                # orient_node["pitch_deg"] = -self.dd.simdata["Plane Pitch Degrees"] * r2d
                # orient_node["yaw_deg"] = self.dd.simdata["Plane Heading Degrees True"] * r2d
                # imu_node["p_rps"] = -self.dd.simdata["Rotation Velocity Body Z"]
                # imu_node["q_rps"] = -self.dd.simdata["Rotation Velocity Body X"]
                # imu_node["r_rps"] = -self.dd.simdata["Rotation Velocity Body Y"]
                # imu_node["ax_mps2"] = -self.dd.simdata["Acceleration Body Z"] * ft2m  # fixme: may not include "g"
                # imu_node["ay_mps2"] = -self.dd.simdata["Acceleration Body X"] * ft2m
                # imu_node["az_mps2"] = -self.dd.simdata["Acceleration Body Y"] * ft2m
                # vel_node["airspeed_kt"] = self.dd.simdata["Airspeed Indicated"]
                # airdata_node["airspeed_kt"] = self.dd.simdata["Airspeed Indicated"]
                # airdata_node["temp_C"] = self.dd.simdata["Total Air Temperature"]
                # airdata_node["ref_pressure_inhg"] = self.dd.simdata["Barometer Pressure"]