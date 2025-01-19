from math import pi
from lib.props import pos_node, vel_node, att_node, fcs_node

# import navpy
# from lib.constants import mps2kt, r2d

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

        self.prop_rotation_angle_deg = 0

        print("Searching for x-plane on the LAN...")
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

            self.msl_name = "sim/flightmodel/position/elevation"
            self.agl_name = "sim/flightmodel/position/y_agl"
            self.xp.AddDataRef(self.msl_name, freq=10)
            self.xp.AddDataRef(self.agl_name, freq=10)

        if self.xp_ip is not None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self):
        if self.xp_ip is None:
            return
        self.send()
        self.receive()

    def send_data_ref(self, name, value):
        msg = struct.pack('<4sxf500s', b'DREF',
                          value,
                          name.encode('utf-8'))
        self.sock.sendto(msg, (self.xp_ip, self.xp_port))

    def send(self):
        # print("Hey, switch to xp.writeDataRef()!")

        if self.sock is not None:
            # lla = navpy.ned2lla(state.pos_ned, self.lat_deg, self.lon_deg, self.altitude_m)
            # Drive the external MSFS visual system
            lat_deg = pos_node.getDouble("lat_geod_deg")
            lon_deg = pos_node.getDouble("long_gc_deg")
            alt_m = pos_node.getDouble("geod_alt_m")
            alt_ft = alt_m * m2ft
            phi_deg = att_node.getDouble("phi_deg")
            the_deg = att_node.getDouble("theta_deg")
            psi_deg = att_node.getDouble("psi_deg")
            vc = vel_node.getDouble("vc_kts")
            vd = vel_node.getDouble("vd_mps") * m2ft

            # engine power (estimate/hack) // does not work :-(
            self.send_data_ref("sim/operation/override/override_engine_forces", 1)
            self.send_data_ref("sim/flightmodel/engine/ENGN_power", fcs_node.getDouble("posThrottle_nd")*100)

            # https://www.siminnovations.com/xplane/dataref/?name=sim%2Fcockpit&type=float&writable=y&units=&description=&submit=Search

            # visible ailerons and flaps
            self.send_data_ref("sim/operation/override/override_control_surfaces", 1)
            self.send_data_ref("sim/flightmodel/controls/wing3l_ail1def", fcs_node.getDouble("posAil_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing3r_ail1def", -fcs_node.getDouble("posAil_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing2l_fla1def", fcs_node.getDouble("posFlap_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing2r_fla1def", fcs_node.getDouble("posFlap_deg"))

            # not tested elevator/rudder ...
            # self.send_data_ref("sim/aircraft/parts/acf_elev", sim.fdm['fcs/left-aileron-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab1_elv1def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab1_elv2def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab2_elv1def", sim.fdm['fcs/elevator-pos-norm'])
            # self.send_data_ref("sim/flightmodel/controls/hstab2_elv2def", sim.fdm['fcs/elevator-pos-norm'])

            # prop disk
            self.send_data_ref("sim/flightmodel2/engines/prop_disc/override", 1)
            self.send_data_ref("sim/flightmodel2/engines/prop_is_disc", 1)
            self.prop_rotation_angle_deg = (self.prop_rotation_angle_deg - 19) % 360.0
            self.send_data_ref("sim/flightmodel2/engines/prop_rotation_angle_deg[0]", self.prop_rotation_angle_deg)

            # sound (sadly this is a readonly data ref in x-plane)
            # print("engine rotation speed:", fcs_node.getDouble("posThrottle_nd")*2700*0.1072)

            self.send_data_ref("sim/flightmodel2/engines/engine_rotation_speed_rad_sec",
                               fcs_node.getDouble("posThrottle_nd")*2700*0.1072)

            #print(lat_rad, lon_rad, alt_ft, phi, the, psi)
            self.send_data_ref("sim/cockpit/gyros/phi_ind_ahars_pilot_deg", phi_deg)
            self.send_data_ref("sim/cockpit/gyros/the_ind_ahars_pilot_deg", the_deg)
            self.send_data_ref("sim/cockpit/gyros/psi_ind_ahars_pilot_degm", psi_deg) # fixme: sending true, but calling it mag
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

    def receive(self):
        values = self.xp.GetValues()
        if self.msl_name in values and self.agl_name in values:
            msl = values[self.msl_name]
            agl = values[self.agl_name]
            # print(values)
            ground_elev_m = (msl - agl) - 4.7*ft2m
            # print("AGL (ft):", agl*m2ft, "Ground (ft):", ground_elev_ft)
            pos_node.setDouble("visual_terrain_elevation_m", ground_elev_m)
