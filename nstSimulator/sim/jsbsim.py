# -*- coding: utf-8 -*-

'''
Copyright (c) 2016 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
Modifications: Curtis Olson
'''

#%% JSBSim

from math import sqrt
import numpy as np
import time

import jsbsim as jsb    # pip install jsbsim

from ..utils.constants import d2r, ft2m, g, m2ft, r2d
from .lib.props import accel_node, aero_node, airdata_node, att_node, control_node, engine_node, environment_node, fcs_node, gps_node, imu_node, inceptors_node, mass_node, pos_node, root_node, vel_node

slug2kg = 14.5939029
in2m = 0.0254
lb2N = 4.44822162
hp22W = 745.699872

class JSBSimWrap:
    def __init__ (self, model, pathJSB = '.', dt = 1/200):

        self.model = model
        self.dt = dt
        self.pathJSB = pathJSB
        self.trimmed = False
        self.have_ground_elev = False
        self.terrain_latch = False

        self.fdm = jsb.FGFDMExec(pathJSB, None)
        # self.fdm.load_model_with_paths(self.model, pathJSB, pathJSB, pathJSB, False)
        self.fdm.load_model(self.model)
        self.fdm.set_dt(self.dt)
        self.prop_catalog = self.fdm.get_property_catalog()
        # print("jsbsim wrap:", self.prop_catalog)
        self.fileLog = []

    def setup_initial_conditions(self, lla, hdg_deg, vc_kts=0):
        self.fdm["ic/lat-geod-deg"] = lla[0]
        self.fdm["ic/long-gc-deg"] = lla[1]
        self.fdm["ic/h-sl-ft"] = lla[2]*m2ft
        self.fdm['ic/psi-true-deg'] = hdg_deg
        if vc_kts > 0:
            self.fdm["ic/vc-kts"] = vc_kts
        self.fdm["propulsion/set-running"] = -1

        # seed the pos/hdg output properties so our messages to the visual
        # system have our initial position. (Then we can get a terrain altitude
        # back.)
        pos_node.setDouble("long_gc_deg", lla[1])
        pos_node.setDouble("lat_geod_deg", lla[0])
        pos_node.setDouble("geod_alt_m", lla[2])
        att_node.setDouble("psi_deg", hdg_deg)

    def set_initial_terrain_height(self, terrain_ft):
        print("initial terrain elevation: %.3f" % terrain_ft)
        self.fdm["ic/terrain-elevation-ft"] = terrain_ft
        self.have_ground_elev = True

    def do_trim(self):
        # setting this property invokes the JSBSim trim routine
        print("before trim:")
        print("propeller_rpm", self.fdm['propulsion/engine/propeller-rpm'])
        print("power_hp", self.fdm['propulsion/engine/power-hp'])
        print("power_W", self.fdm['propulsion/engine/power-hp'])
        print("blade_angle", self.fdm[ 'propulsion/engine/blade-angle'])
        print("advance_ratio", self.fdm[ 'propulsion/engine/advance-ratio'])
        print("thrust_lb", self.fdm[ 'propulsion/engine/thrust-lbs'])
        print("thrust_N", self.fdm[ 'propulsion/engine/thrust-lbs'])
        try:
            if self.fdm["ic/vc-kts"] > 0.1:
                print("In air trim")
                self.fdm['simulation/do_simple_trim'] = 0  # In-air trim
            else:
                print("ground trim")
                self.fdm['simulation/do_simple_trim'] = 2  # Ground trim
            self.trimmed = True
        except:
            print("after failed trim:")
            print("propeller_rpm", self.fdm['propulsion/engine/propeller-rpm'])
            print("power_hp", self.fdm['propulsion/engine/power-hp'])
            print("power_W", self.fdm['propulsion/engine/power-hp'])
            print("blade_angle", self.fdm[ 'propulsion/engine/blade-angle'])
            print("advance_ratio", self.fdm[ 'propulsion/engine/advance-ratio'])
            print("thrust_lb", self.fdm[ 'propulsion/engine/thrust-lbs'])
            print("thrust_N", self.fdm[ 'propulsion/engine/thrust-lbs'])
            quit()

    def SetWindNED(self, vWind_mps = [0.0, 0.0, 0.0]):
        self.fdm['atmosphere/wind-north-fps'] = vWind_mps[0] * m2ft
        self.fdm['atmosphere/wind-east-fps'] = vWind_mps[1] * m2ft
        self.fdm['atmosphere/wind-down-fps'] = vWind_mps[2] * m2ft

    def GetWindNED(self):
        return [self.fdm['atmosphere/wind-north-fps'] * ft2m, self.fdm['atmosphere/wind-east-fps'] * ft2m, self.fdm['atmosphere/wind-down-fps'] * ft2m]

    def SetWind(self, vWindMag_mps = 0.0, vWindHeading_deg = 0.0, vWindDown_mps = 0.0):
        self.fdm['atmosphere/wind-mag-fps'] = vWindMag_mps * m2ft
        self.fdm['atmosphere/psiw-rad'] = vWindHeading_deg * d2r
        self.fdm['atmosphere/wind-down-fps'] = vWindDown_mps * m2ft

    # Update the wind in JSBSim for the current altitude
    def UpdateWind(self):
        vWind20_fps = self.fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps']
        h_ft = np.max([self.fdm['position/h-agl-ft'], 0.1])

        # Compute Wind Shear (MIL-DTL-9490E, 3.1.3.7.3.2) to compute
        self.fdm['atmosphere/wind-mag-fps'] = vWind20_fps * (0.46 * np.log10(h_ft) + 0.4)

        self.vWind20_mps = vWind20_fps * ft2m

    def SetTurb(self, turbType = 3, turbSeverity = 0, vWind20_mps = None, vWindHeading_deg = 0.0):
        self.vWind20_mps = vWind20_mps
        # Type
        # 0: ttNone (turbulence disabled)
        # 1: ttStandard
        # 2: ttCulp
        # 3: ttMilspec (Dryden spectrum)
        # 4: ttTustin (Dryden spectrum) - doesn't work!!

        # Severity
        # 3: 10^-2 (Light)
        # 4: 10^-3 (Moderate)
        # 6: 10^-5 (Severe)

        self.fdm['atmosphere/turb-type'] = turbType
        self.fdm['atmosphere/turbulence/milspec/severity'] = turbSeverity
        self.fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps'] = vWind20_mps * m2ft
        self.fdm['atmosphere/psiw-rad'] = vWindHeading_deg * d2r
        self.UpdateWind()

    def UpdateTerrainElevation(self):
        # honor visual system ground elevation if set
        xp_ground_m = pos_node.getDouble("xp_terrain_elevation_m")
        vis_ground_m = pos_node.getDouble("visual_terrain_elevation_m")
        if xp_ground_m > 0:
            ground_m = xp_ground_m
        elif vis_ground_m > 0:
            ground_m = vis_ground_m
        else:
            return

        current_ground_m = self.fdm["position/terrain-elevation-asl-ft"] * ft2m

        if ground_m > 0:
            if not self.trimmed:
                self.set_initial_terrain_height(ground_m * m2ft)
            else:
                if self.terrain_latch:
                    # set terrain height directly
                    self.fdm["position/terrain-elevation-asl-ft"] = ground_m * m2ft
                    # print("terrain elevation latched, terrain set to: %.3f" % (ground_m * m2ft))
                else:
                    # slew ground elevation slowly to avoid chaos
                    max_delta = 0.02
                    diff = ground_m - current_ground_m
                    if diff < -max_delta: diff = -max_delta
                    if diff > max_delta: diff = max_delta
                    self.fdm["position/terrain-elevation-asl-ft"] = (current_ground_m + diff) * m2ft
                    # print("ground_m:", ground_m, "cur ground_m:", current_ground_m)
                    # print("terrain elevation not latched, set to: %.3f" % ((current_ground_m + diff) * m2ft))
                    if abs(current_ground_m - ground_m) < 0.1:
                        self.terrain_latch = True

    def RunTo(self, time_s, updateWind = None):
        # honor visual system ground elevation if set
        vis_ground_m = pos_node.getDouble("visual_terrain_elevation_m")
        if vis_ground_m > 0:
            self.fdm["position/terrain-elevation-asl-ft"] = vis_ground_m * m2ft

        if updateWind:
            self.UpdateWind()
        while (self.fdm.get_sim_time() <= time_s): # Run the FDM
            self.fdm.run()

    def RunSteps(self, steps, updateWind = None):
        # update control inputs
        self.fdm['fcs/throttle-cmd-norm'] = control_node.getDouble("throttle")
        self.fdm['fcs/aileron-cmd-norm'] = control_node.getDouble("aileron")
        self.fdm['fcs/elevator-cmd-norm'] = control_node.getDouble("elevator")
        self.fdm['fcs/pitch-trim-cmd-norm'] = control_node.getDouble("elevator_trim")
        self.fdm['fcs/rudder-cmd-norm'] = -control_node.getDouble("rudder")
        self.fdm['fcs/flap-cmd-norm'] = inceptors_node.getDouble("cmdFlap_norm")
        self.fdm['fcs/left-brake-cmd-norm'] = control_node.getDouble("brake_left")
        self.fdm['fcs/right-brake-cmd-norm'] = control_node.getDouble("brake_right")

        if updateWind:
            self.UpdateWind()

        for i in range(steps):
            self.fdm.run()

    def update(self, steps, updateWind=True):
        self.UpdateTerrainElevation()
        if not self.trimmed and self.have_ground_elev:
            self.do_trim()
        if self.trimmed:
            self.RunSteps(steps, updateWind)
            self.PublishProps()

    # estimate the 'ideal' magnetometer reading in body coordinates
    def EstMagBody(self, lat_deg, lon_deg, phi_rad, the_rad, psi_rad):
        import geomag  # pip install geomag
        import navpy
        gm = geomag.geomag.GeoMag()
        mag = gm.GeoMag(lat_deg, lon_deg)
        mag_ned = np.array( [mag.bx, mag.by, mag.bz] )
        norm = np.linalg.norm(mag_ned)
        mag_ned /= norm
        N2B = navpy.angle2dcm(psi_rad, the_rad, phi_rad, input_unit='rad')
        mag_body = N2B.dot(mag_ned)
        norm = np.linalg.norm(mag_body)
        mag_body /= norm
        # print("  mag ned:", mag_ned, "body:", mag_body)
        return mag_body

    def PublishProps(self):
        root_node.setDouble("sim_time_sec", self.fdm['simulation/sim-time-sec'])

        # Mass Properties
        mass_node.setDouble("mass_kg", self.fdm['inertia/mass-slugs'] * slug2kg)
        mass_node.setDouble("weight_lb", self.fdm['inertia/weight-lbs'])
        mass_node.setDouble("weight_N", self.fdm['inertia/weight-lbs'] * lb2N)

        mass_node.setDouble("weightFuel_lb", self.fdm['propulsion/total-fuel-lbs'])
        mass_node.setDouble("weightFuel_N", self.fdm['propulsion/total-fuel-lbs'] * lb2N)

        mass_node.setDouble("rCgX_Bf_m", self.fdm['inertia/cg-x-in'] * in2m)
        mass_node.setDouble("rCgY_Bf_m", self.fdm['inertia/cg-y-in'] * in2m)
        mass_node.setDouble("rCgZ_Bf_m", self.fdm['inertia/cg-z-in'] * in2m)

        mass_node.setDouble("inertiaXX_kgm2", self.fdm['inertia/ixx-slugs_ft2'] * slug2kg * ft2m**2)
        mass_node.setDouble("inertiaYY_kgm2", self.fdm['inertia/iyy-slugs_ft2'] * slug2kg * ft2m**2)
        mass_node.setDouble("inertiaZZ_kgm2", self.fdm['inertia/izz-slugs_ft2'] * slug2kg * ft2m**2)
        mass_node.setDouble("inertiaXY_kgm2", self.fdm['inertia/ixy-slugs_ft2'] * slug2kg * ft2m**2)
        mass_node.setDouble("inertiaYZ_kgm2", self.fdm['inertia/iyz-slugs_ft2'] * slug2kg * ft2m**2)
        mass_node.setDouble("inertiaXZ_kgm2", self.fdm['inertia/ixz-slugs_ft2'] * slug2kg * ft2m**2)

        # Environment
        environment_node.setDouble("aGrav_mps2", self.fdm['accelerations/gravity-ft_sec2'] * ft2m)

        environment_node.setDouble("rho_kgpm3", self.fdm['atmosphere/rho-slugs_ft3'] * slug2kg * m2ft**3)
        environment_node.setDouble("temp_C", (self.fdm['atmosphere/T-R'] - 491.67) / 1.8)
        environment_node.setDouble("pres_Pa", self.fdm['atmosphere/P-psf'] * lb2N * m2ft**2)

        environment_node.setDouble("psiWind_rad", self.fdm['atmosphere/psiw-rad'])
        environment_node.setDouble("vWindMag_mps", self.fdm['atmosphere/wind-mag-fps'] * ft2m)
        environment_node.setDouble("vWindMag20_mps", self.fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps'] * ft2m) # Wind at 20ft AGL
        environment_node.setDouble("turbSeverity", self.fdm['atmosphere/turbulence/milspec/severity']) # Wind at 20ft AGL

        environment_node.setDouble("pTurb_rps", self.fdm['atmosphere/p-turb-rad_sec'])
        environment_node.setDouble("qTurb_rps", self.fdm['atmosphere/q-turb-rad_sec'])
        environment_node.setDouble("rTurb_rps", self.fdm['atmosphere/r-turb-rad_sec'])

        environment_node.setDouble("vTurbN_mps", self.fdm['atmosphere/turb-north-fps'] * ft2m)
        environment_node.setDouble("vTurbE_mps", self.fdm['atmosphere/turb-east-fps'] * ft2m)
        environment_node.setDouble("vTurbD_mps", self.fdm['atmosphere/turb-down-fps'] * ft2m)

        environment_node.setDouble("vWindN_mps", self.fdm['atmosphere/total-wind-north-fps'] * ft2m)
        environment_node.setDouble("vWindE_mps", self.fdm['atmosphere/total-wind-east-fps'] * ft2m)
        environment_node.setDouble("vWindD_mps", self.fdm['atmosphere/total-wind-down-fps'] * ft2m)

        # Accelerations
        accel_node.setDouble("pDot_rps2", self.fdm['accelerations/pdot-rad_sec2'])
        accel_node.setDouble("qDot_rps2", self.fdm['accelerations/qdot-rad_sec2'])
        accel_node.setDouble("rDot_rps2", self.fdm['accelerations/rdot-rad_sec2'])
        accel_node.setDouble("uDot_mps2", self.fdm['accelerations/udot-ft_sec2'] * ft2m)
        accel_node.setDouble("vDot_mps2", self.fdm['accelerations/vdot-ft_sec2'] * ft2m)
        accel_node.setDouble("wDot_mps2", self.fdm['accelerations/wdot-ft_sec2'] * ft2m)

        # Aerodynamics
        aero_node.setDouble("alpha_deg", self.fdm['aero/alpha-deg'])
        aero_node.setDouble("beta_deg", self.fdm['aero/beta-deg'])
        aero_node.setDouble("alphaDot_dps", self.fdm['aero/alphadot-deg_sec'])
        aero_node.setDouble("beta_deg", self.fdm['aero/beta-deg'])
        aero_node.setDouble("betaDot_dps", self.fdm['aero/betadot-deg_sec'])

        aero_coef_elements = self.fdm.query_property_catalog('aero/coefficient')
        aero_coef_elements = aero_coef_elements.replace(' (R)', '')
        aero_coef_elements = aero_coef_elements.replace(' (RW)', '')
        aero_coef_elements = aero_coef_elements.split('\n')
        for elem in aero_coef_elements:
            if elem != '':
                aero_node.setDouble(elem, self.fdm[elem])

        # Velocities
        vel_node.setDouble("vtrue_mps", self.fdm['velocities/vtrue-fps'] * ft2m)
        vel_node.setDouble("vtrue_kts", self.fdm['velocities/vtrue-kts'])
        vel_node.setDouble("vc_mps", self.fdm['velocities/vc-fps'] * ft2m)
        vel_node.setDouble("vc_kts", self.fdm['velocities/vc-kts'])
        vel_node.setDouble("mach_nd", self.fdm['velocities/mach'])
        vel_node.setDouble("hDot_mps", self.fdm['velocities/h-dot-fps'] * ft2m)
        vel_node.setDouble("vn_mps", self.fdm['velocities/v-north-fps'] * ft2m)
        vel_node.setDouble("ve_mps", self.fdm['velocities/v-east-fps'] * ft2m)
        vel_node.setDouble("vd_mps", self.fdm['velocities/v-down-fps'] * ft2m)
        vel_node.setDouble("u_mps", self.fdm['velocities/u-fps'] * ft2m)
        vel_node.setDouble("v_mps", self.fdm['velocities/v-fps'] * ft2m)
        vel_node.setDouble("w_mps", self.fdm['velocities/w-fps'] * ft2m)
        vel_node.setDouble("vground_mps", self.fdm['velocities/vg-fps'] * ft2m)

        # Attitude
        att_node.setDouble("phi_deg", self.fdm['attitude/phi-deg'])
        att_node.setDouble("theta_deg", self.fdm['attitude/theta-deg'])
        att_node.setDouble("psi_deg", self.fdm['attitude/psi-deg'])
        att_node.setDouble("gamma_deg", self.fdm['flight-path/gamma-deg'])
        if vel_node.getDouble("vground_mps") > 1:
            att_node.setDouble("ground_track_deg", self.fdm['flight-path/psi-gt-rad'] * r2d)

        # Propulsion
        engine_node.setDouble("propeller_rpm", self.fdm['propulsion/engine/propeller-rpm'])
        engine_node.setDouble("power_hp", self.fdm['propulsion/engine/power-hp'])
        engine_node.setDouble("power_W", self.fdm['propulsion/engine/power-hp'] * hp22W)
        engine_node.setDouble("blade_angle", self.fdm[ 'propulsion/engine/blade-angle'])
        engine_node.setDouble("advance_ratio", self.fdm[ 'propulsion/engine/advance-ratio'])
        engine_node.setDouble("thrust_lb", self.fdm[ 'propulsion/engine/thrust-lbs'])
        engine_node.setDouble("thrust_N", self.fdm[ 'propulsion/engine/thrust-lbs'] * lb2N)

        # Flight Control System (Actuators)
        if 'fcs/cmdAil_deg (RW)' in self.prop_catalog:
            # alternate convention
            fcs_node.setDouble("cmdAil_deg", self.fdm['fcs/cmdAil_deg'])
            fcs_node.setDouble("posAil_deg", self.fdm['fcs/posAil_deg'])
            fcs_node.setDouble("cmdElev_deg", self.fdm['fcs/cmdElev_deg'])
            fcs_node.setDouble("posElev_deg", self.fdm['fcs/posElev_deg'])
            fcs_node.setDouble("cmdRud_deg", self.fdm['fcs/cmdRud_deg'])
            fcs_node.setDouble("posRud_deg", self.fdm['fcs/posRud_deg'])
            fcs_node.setDouble("cmdFlap_deg", self.fdm['fcs/cmdFlap_deg'])
            fcs_node.setDouble("posFlap_deg", self.fdm['fcs/posFlap_deg'])
            fcs_node.setDouble("posFlap_nd",  self.fdm['fcs/flap-pos-norm'])
            fcs_node.setDouble("cmdThrottle_nd", self.fdm['fcs/throttle-cmd-norm'])
            fcs_node.setDouble("posThrottle_nd", self.fdm['fcs/throttle-pos-norm'])
            fcs_node.setDouble("cmdBrakeLeft_nd", self.fdm['fcs/left-brake-cmd-norm'])
            fcs_node.setDouble("cmdBrakeRight_nd", self.fdm['fcs/right-brake-cmd-norm'])
        else:
            # FlightGear convention
            fcs_node.setDouble("cmdAil_norm", self.fdm['fcs/aileron-cmd-norm'])
            fcs_node.setDouble("posAil_norm", self.fdm['fcs/left-aileron-pos-norm'])
            fcs_node.setDouble("cmdElev_norm", self.fdm['fcs/elevator-cmd-norm'])
            fcs_node.setDouble("posElev_norm", self.fdm['fcs/elevator-pos-norm'])
            fcs_node.setDouble("cmdRud_norm", self.fdm['fcs/rudder-cmd-norm'])
            fcs_node.setDouble("posRud_norm", self.fdm['fcs/rudder-pos-norm'])
            fcs_node.setDouble("cmdFlap_norm", self.fdm['fcs/flap-cmd-norm'])
            fcs_node.setDouble("posFlap_norm", self.fdm['fcs/flap-pos-norm'])
            fcs_node.setDouble("cmdThrottle_nd", self.fdm['fcs/throttle-cmd-norm'])
            fcs_node.setDouble("posThrottle_nd", self.fdm['fcs/throttle-pos-norm'])
            fcs_node.setDouble("cmdBrakeLeft_nd", self.fdm['fcs/left-brake-cmd-norm'])
            fcs_node.setDouble("cmdBrakeRight_nd", self.fdm['fcs/right-brake-cmd-norm'])

        # Positions
        pos_node.setDouble("long_gc_deg", self.fdm['position/long-gc-deg'])
        pos_node.setDouble("lat_geod_deg", self.fdm['position/lat-geod-deg'])
        pos_node.setDouble("geod_alt_m", self.fdm['position/geod-alt-ft'] * ft2m)
        pos_node.setDouble("h_sl_m", self.fdm['position/h-sl-ft'] * ft2m)
        pos_node.setDouble("h_agl_m", self.fdm['position/h-agl-ft'] * ft2m)
        pos_node.setBool("wow", self.fdm['gear/wow']) # for lack of a better place for now

        millis = int(round(self.fdm['simulation/sim-time-sec']*1000))

        # airdata
        airdata_node.setUInt("millis", millis)
        airdata_node.setDouble("baro_press_pa", self.fdm['atmosphere/P-psf'] * lb2N * m2ft**2)
        airdata_node.setDouble("diff_press_pa", 0)
        airdata_node.setDouble("air_temp_C", (self.fdm['atmosphere/T-R'] - 491.67) / 1.8)
        airdata_node.setDouble("airspeed_mps", self.fdm['velocities/vc-fps'] * ft2m)
        airdata_node.setDouble("altitude_m", self.fdm['position/h-sl-ft'] * ft2m)
        # airdata_node.setDouble("altitude_agl_m", self.fdm['position/h-agl-ft'] * ft2m)
        # airdata_node.setDouble("altitude_true_m", self.fdm['position/h-sl-ft'] * ft2m)
        # airdata_node.setDouble("altitude_ground_m", self.fdm["position/terrain-elevation-asl-ft"] * ft2m)
        # airdata_node.setUInt("is_airborne", self.fdm['position/h-agl-ft'] >= 10.0)
        # airdata_node.setUInt("flight_timer_millis", millis)
        # airdata_node.setDouble("wind_dir_deg", self.fdm['atmosphere/psiw-rad'] * rad2deg)
        # airdata_node.setDouble("wind_speed_mps", self.fdm['atmosphere/wind-mag-fps'] * ft2m)
        # if self.fdm['velocities/vc-fps'] > 10:
        #     airdata_node.setDouble("pitot_scale_factor", self.fdm['velocities/vtrue-fps'] / self.fdm['velocities/vc-fps'] )
        # else:
        #     airdata_node.setDouble("pitot_scale_factor", 1.0)
        airdata_node.setUInt("error_count", 0)

        # gps
        gps_node.setUInt("millis", millis)
        gps_node.setUInt64("unix_usec", int(round(time.time() * 1000000)))
        gps_node.setUInt("num_sats", 9)
        gps_node.setUInt("status", 3)
        gps_node.setDouble("longitude_raw", int(round(self.fdm['position/long-gc-deg'] * 10000000)))
        gps_node.setDouble("latitude_raw", int(round(self.fdm['position/lat-geod-deg'] * 10000000)))
        gps_node.setDouble("altitude_m", self.fdm['position/geod-alt-ft'] * ft2m)
        gps_node.setDouble("vn_mps", self.fdm['velocities/v-north-fps'] * ft2m)
        gps_node.setDouble("ve_mps", self.fdm['velocities/v-east-fps'] * ft2m)
        gps_node.setDouble("vd_mps", self.fdm['velocities/v-down-fps'] * ft2m)

         # imu
        mag_body = self.EstMagBody(self.fdm['position/lat-geod-deg'], self.fdm['position/long-gc-deg'], self.fdm['attitude/phi-rad'], self.fdm['attitude/theta-rad'], self.fdm['attitude/psi-rad'])
        imu_node.setUInt("millis", millis)
        imu_node.setDouble("ax_raw", -self.fdm['accelerations/Nx'] * g)
        imu_node.setDouble("ay_raw", -self.fdm['accelerations/Ny'] * g)
        imu_node.setDouble("az_raw", self.fdm['accelerations/Nz'] * g)
        imu_node.setDouble("hx_raw", mag_body[0])
        imu_node.setDouble("hy_raw", mag_body[1])
        imu_node.setDouble("hz_raw", mag_body[2])
        imu_node.setDouble("ax_mps2", -self.fdm['accelerations/Nx'] * g)
        imu_node.setDouble("ay_mps2", -self.fdm['accelerations/Ny'] * g)
        imu_node.setDouble("az_mps2", self.fdm['accelerations/Nz'] * g)
        imu_node.setDouble("p_rps", self.fdm['velocities/p-rad_sec'])
        imu_node.setDouble("q_rps", self.fdm['velocities/q-rad_sec'])
        imu_node.setDouble("r_rps", self.fdm['velocities/r-rad_sec'])
        imu_node.setDouble("hx", mag_body[0])
        imu_node.setDouble("hy", mag_body[1])
        imu_node.setDouble("hz", mag_body[2])
        imu_node.setDouble("temp_C", 15)

    def __del__(self):
        del self.fdm


# %%
