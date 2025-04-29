from math import pi
from nstSimulator.sim.lib.props import aero_node, att_node, engine_node,  fcs_node, pos_node, vel_node

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

        self.vc_filt = 0
        self.vtrue_filt = 0
        self.power_filt = 0
        self.prop_rotation_angle_deg = 0

        self.timer = 0

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

    def update(self, dt):
        if self.xp_ip is None:
            return
        self.send()
        self.receive(dt)

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
            self.vc_filt = 0.9 * self.vc_filt + 0.1 * vc
            vtrue = vel_node.getDouble("vtrue_kts")
            self.vtrue_filt = 0.9 * self.vtrue_filt + 0.1 * vtrue
            vd = vel_node.getDouble("vd_mps") * m2ft

            # engine power (estimate/hack)
            power = fcs_node.getDouble("posThrottle_nd")*232207
            self.power_filt = 0.95 * self.power_filt + 0.05 * power
            self.send_data_ref("sim/operation/override/override_engine_forces", 1)
            self.send_data_ref("sim/flightmodel/engine/ENGN_power[0]", self.power_filt)
            self.send_data_ref("sim/cockpit2/engine/indicators/engine_speed_rpm[0]", engine_node.getDouble("propeller_rpm"))
            # print("rpm:", engine_node.getFloat("propeller_rpm"))

            # https://www.siminnovations.com/xplane/dataref/?name=sim%2Fcockpit&type=float&writable=y&units=&description=&submit=Search


            # visible ailerons and flaps
            self.send_data_ref("sim/operation/override/override_control_surfaces", 1)
            self.send_data_ref("sim/flightmodel/controls/wing3l_ail1def", fcs_node.getDouble("posAil_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing3r_ail1def", -fcs_node.getDouble("posAil_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing2l_fla1def", fcs_node.getDouble("posFlap_deg"))
            self.send_data_ref("sim/flightmodel/controls/wing2r_fla1def", fcs_node.getDouble("posFlap_deg"))

            # elevator/rudder ...
            self.send_data_ref("sim/flightmodel/controls/hstab1_elv1def", fcs_node.getDouble("posElev_deg"))
            self.send_data_ref("sim/flightmodel/controls/vstab1_rud1def", -fcs_node.getDouble("posRud_deg"))

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
            self.send_data_ref("sim/flightmodel/position/indicated_airspeed", self.vc_filt)
            self.send_data_ref("sim/cockpit2/gauges/indicators/true_airspeed_kts_pilot", self.vtrue_filt)
            self.send_data_ref("sim/flightmodel/misc/h_ind", alt_ft)
            self.send_data_ref("sim/flightmodel/position/vh_ind_fpm", -vd*60)
            self.send_data_ref("sim/cockpit2/gauges/indicators/slip_deg", -aero_node.getDouble("beta_deg"))

            msg = struct.pack('<4sxidddfff', b'VEHX',
                  0,              # The index of the airplane you want to control.
                  lat_deg,        # latitude, in degrees
                  lon_deg,        # longitude, in degrees
                  alt_m,          # elevation above sea level, in meters
                  psi_deg,        # heading, degrees true
                  the_deg,        # pitch, degrees
                  phi_deg)        # roll, degrees
            self.sock.sendto(msg, (self.xp_ip, self.xp_port))

    def receive(self, dt):
        values = self.xp.GetValues()
        if self.msl_name in values and self.agl_name in values:
            self.timer += dt
            if self.timer > 1:
                msl = values[self.msl_name]
                agl = values[self.agl_name]
                # print(values)
                ground_elev_m = (msl - agl) - 4.7*ft2m
                # print("AGL (ft):", agl*m2ft, "Ground (ft):", ground_elev_ft)
                pos_node.setDouble("xp_terrain_elevation_m", ground_elev_m)
        else:
            self.timer = 0