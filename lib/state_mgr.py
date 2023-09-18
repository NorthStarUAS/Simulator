# manage construction of the state vector by name.  Intended to help
# avoid getting lost in our index offsets as we play around with
# different state parameters.

from math import atan2, sin, sqrt
import numpy as np

from lib.constants import gravity, d2r
from lib import quaternion

class StateManager():
    def __init__(self, vehicle="wing"):
        self.vehicle = vehicle
        self.input_states = []
        self.internal_states = []
        self.output_states = []
        self.state_list = []
        self.dt = None
        self.airborne_thresh_mps = 10 # default for small fixed wing drone
        self.land_thresh_mps = 7      # default for small fixed wing drone
        self.airspeed_mps = 0
        self.wn_filt = 0
        self.we_filt = 0
        self.wd_filt = 0

        self.aileron = 0
        self.elevator = 0
        self.rudder = 0
        self.flaps = 0
        self.throttle = 0

        self.qbar = 0
        self.lift = 0
        self.Cl = 0
        self.drag = 0
        self.Cd = 0
        self.thrust = 0
        self.have_alpha = False
        self.alpha = 0
        self.alpha_prev1 = 0
        self.alpha_prev2 = 0
        self.beta = 0
        self.beta_prev1 = 0
        self.beta_prev2 = 0
        self.flying = False
        self.ground_alt = None
        self.g_ned = np.array( [0.0, 0.0, gravity] )
        self.g_body = np.array( [0.0, 0.0, 0.0] )
        self.v_ned = np.array( [0.0, 0.0, 0.0] )
        self.v_body = np.array( [0.0, 0.0, 0.0] )

        self.gyros = np.array( [0.0, 0.0, 0.0] )
        self.gyros_prev1 = np.array( [0.0, 0.0, 0.0] )
        self.gyros_prev2 = np.array( [0.0, 0.0, 0.0] )
        self.accels = np.array( [0.0, 0.0, 0.0] )
        self.accels_prev1 = np.array( [0.0, 0.0, 0.0] )

        self.ned2body = quaternion.eul2quat( 0, 0, 0 )

    def set_state_names(self, input_states, internal_states, output_states):
        self.input_states = input_states
        self.internal_states = internal_states
        self.output_states = output_states
        self.state_list = self.input_states + self.internal_states + self.output_states

    def get_state_index(self, state_name_list):
        result = []
        for n in state_name_list:
            try:
                index = self.state_list.index(n)
                result.append(index)
            except:
                print("Request for non-existent state name")
                result.append(None)
        return result

    def set_dt(self, dt):
        self.dt = dt

    def set_is_flying_thresholds(self, airborne_thresh_mps, land_thresh_mps):
        if land_thresh_mps >= airborne_thresh_mps:
            print("land threshold must be lower than airborne threshold for hysterisis.")
        self.airborne_thresh_mps = airborne_thresh_mps
        self.land_thresh_mps = land_thresh_mps

    def set_time(self, time):
        self.time = time

    def set_throttle(self, throttle):
        self.throttle = throttle
        if self.throttle < 0: self.throttle = 0
        if self.throttle > 1: self.throttle = 1
        # max thrust is 0.75 gravity, so we can't quite hover on full power
        self.thrust = sqrt(self.throttle) * 0.75 * abs(gravity)

    def set_flight_surfaces(self, aileron, elevator, rudder, flaps=0):
        self.aileron = aileron
        self.elevator = elevator
        self.rudder = rudder
        self.flaps = flaps
        if self.aileron < -1: self.aileron = -1
        if self.aileron > 1: self.aileron = 1
        if self.elevator < -1: self.elevator = -1
        if self.elevator > 1: self.elevator = 1
        if self.rudder < -1: self.rudder = -1
        if self.rudder > 1: self.rudder = 1
        if self.flaps < 0: self.flaps = 0
        if self.flaps > 1: self.flaps = 1

    def set_motors(self, motors):
        self.motors = motors.copy()

    def set_orientation(self, phi_rad, the_rad, psi_rad):
        self.phi_rad = phi_rad
        self.the_rad = the_rad
        self.psi_rad = psi_rad
        self.ned2body = quaternion.eul2quat( phi_rad, the_rad, psi_rad )

    def set_airdata(self, airspeed_mps, alpha_rad=None, beta_rad=None):
        self.airspeed_mps = airspeed_mps
        self.qbar = 0.5 * self.airspeed_mps**2
        self.v_body[0] = airspeed_mps
        self.alpha_prev2 = self.alpha_prev1
        self.alpha_prev1 = self.alpha
        if alpha_rad is not None:
            self.have_alpha = True
            self.alpha = alpha_rad
            self.v_body[2] = airspeed_mps * sin(alpha_rad)
        self.beta_prev2 = self.beta_prev1
        self.beta_prev1 = self.beta
        if beta_rad is not None:
            self.beta = beta_rad
            self.v_body[1] = airspeed_mps * sin(beta_rad)
        # print("rudder:", self.rudder, "beta:", self.beta, "vby:", self.v_body[1])
        # print("alpha:", self.alpha*r2d, "v_body:", self.v_body)

    def set_wind(self, wn, we):
        self.wn_filt = 0.95 * self.wn_filt + 0.05 * wn
        self.we_filt = 0.95 * self.we_filt + 0.05 * we

    def set_gyros(self, gyros):
        self.gyros_prev2 = self.gyros_prev1.copy()
        self.gyros_prev1 = self.gyros.copy()
        self.gyros = gyros

    def set_accels(self, accels):
        self.accels_prev1 = self.accels.copy()
        self.accels = accels

    def set_ned_velocity(self, vn, ve, vd, wn, we, wd):
        # store NED velocity, corrected to remove wind effects
        self.wn_filt = 0.95 * self.wn_filt + 0.05 * wn
        self.we_filt = 0.95 * self.we_filt + 0.05 * we
        self.wd_filt = 0.95 * self.wd_filt + 0.05 * wd
        self.v_ned = np.array( [vn + wn, ve + we, vd + wd] )

    def set_body_velocity(self, v_body):
        self.v_body = v_body

    # update attitude
    def update_attitude(self):
        # attitude: integrate rotational rates
        delta_rot = self.gyros * self.dt
        rot_body = quaternion.eul2quat(delta_rot[0], delta_rot[1], delta_rot[2])
        self.ned2body = quaternion.multiply(self.ned2body, rot_body)
        self.phi_rad, self.the_rad, self.psi_rad = quaternion.quat2eul(self.ned2body)

    # compute gravity vector in body frame
    def update_gravity_body(self):
        self.g_body = quaternion.transform(self.ned2body, self.g_ned)

    # accels = (ax, ay, az), g_body = (gx, gy, gz)
    def update_body_velocity(self):
        self.v_body += (self.accels - self.g_body) * self.dt
        print(self.g_ned, self.g_body, self.accels - self.g_body, self.v_body)

    def update_airdata(self, alpha_rad, beta_rad):
        # self.accels[0] = ax_mps2
        self.v_body[0] += (self.accels[0] - self.g_body[0])  * self.dt
        self.airspeed_mps = self.v_body[0]
        self.qbar = 0.5 * self.airspeed_mps**2
        self.alpha_prev2 = self.alpha_prev1
        self.alpha_prev1 = self.alpha
        self.alpha = alpha_rad
        self.v_body[2] = self.v_body[0] * sin(alpha_rad)
        self.beta_prev2 = self.beta_prev1
        self.beta_prev1 = self.beta
        self.beta = beta_rad
        self.v_body[1] = self.v_body[0] * sin(beta_rad)

        # hey, estimate ay, az accels! (and make the new vel official) and FIXME!
        # print("FIXME!")
        # self.accels[1] = (vby - self.v_body[1]) / self.dt # + self.g_body[1]
        # self.v_body[1] = vby
        # self.accels[2] = (vbz - self.v_body[2]) / self.dt # + self.g_body[2]
        # self.v_body[2] = vby

    def update_airdata_from_accels(self):
        # todo: use observed min/max range from model

        self.airspeed_mps = self.v_body[0]
        self.qbar = 0.5 * self.airspeed_mps**2

        # try to clamp side/vertical velocities from getting crazy
        cutoff = 0.3
        if abs(self.v_body[1] / self.v_body[0]) > cutoff:
            self.v_body[1] = np.sign(self.v_body[1]) * abs(self.v_body[0]) * cutoff
        if abs(-self.v_body[2] / self.v_body[0]) > cutoff:
            self.v_body[2] = np.sign(self.v_body[2]) * abs(self.v_body[0]) * cutoff

        # alpha and beta from body frame velocity
        self.alpha_prev2 = self.alpha_prev1
        self.alpha_prev1 = self.alpha
        self.beta_prev2 = self.beta_prev1
        self.beta_prev1 = self.beta
        # max = 20 * d2r
        self.alpha = atan2( self.v_body[2], self.v_body[0] )
        # if abs(self.alpha) > max:
        #     self.alpha = np.sign(self.alpha) * max
        self.beta = atan2( self.v_body[1], self.v_body[0] )
        # if abs(self.beta) > max:
        #     self.beta = np.sign(self.beta) * max

    def set_pos(self, lon, lat, alt):
        self.lon = lon
        self.lat = lat
        self.alt = alt
        if self.ground_alt is None or alt < self.ground_alt:
            self.ground_alt = alt

    def is_flying(self):
        if self.vehicle == "wing":
            # ground speed mps (ned)
            gs_mps = sqrt( self.v_ned[0]**2 + self.v_ned[1]**2 )

            # test if we are flying?
            if not self.flying and gs_mps > self.airborne_thresh_mps*0.7 and self.airspeed_mps > self.airborne_thresh_mps:
                print("Start flying @ %.2f" % self.time)
                self.flying = True
            elif self.flying and gs_mps < self.airborne_thresh_mps*1.2 and self.airspeed_mps < self.land_thresh_mps:
                print("Stop flying @ %.2f" % self.time)
                self.flying = False
        elif self.vehicle == "quad":
            if not self.flying and self.alt > self.ground_alt + 2:
                print("Start flying @ %.2f" % self.time)
                self.flying = True
            elif self.flying and self.alt < self.ground_alt + 1:
                print("Stop flying @ %.2f" % self.time)
                self.flying = False
        return self.flying

    # compute body frame of reference values
    def compute_body_frame_values(self, have_alpha_beta=False):
        if self.dt is None:
            print("Did you forget to set dt for this system?")
            return None

        # transformation between NED coordations and body coordinates
        ned2body = quaternion.eul2quat( self.phi_rad, self.the_rad, self.psi_rad )

        if not have_alpha_beta:
            # rotate ned velocity vector into body frame
            self.v_body = quaternion.transform(ned2body, self.v_ned)
            #print("v_ned:", self.v_ned, np.linalg.norm(self.v_ned),
            #      "v_body:", self.v_body, np.linalg.norm(self.v_body))

            # compute alpha and beta from body frame velocity
            self.alpha = atan2( self.v_body[2], self.v_body[0] )
            self.beta = atan2( -self.v_body[1], self.v_body[0] )
            #print("v(body):", v_body, "alpha = %.1f" % (self.alpha/d2r), "beta = %.1f" % (self.beta/d2r))


        # rotate ned gravity vector into body frame
        self.g_body = quaternion.transform(ned2body, self.g_ned)

        # lift, drag, and weight vector estimates

        # is my math correct here? (for drag need grav & body_accel in
        # flight path frame of reference ... I think.

        self.drag = self.accels[0] - self.thrust
        if self.qbar > 10:
            self.Cd = self.drag / self.qbar
        else:
            self.Cd = 0

        self.lift = -self.accels[2]

        if self.qbar > 10:
            self.Cl = self.lift / self.qbar
        else:
            self.Cl = 0

    def gen_state_vector(self, params=None):
        result = []
        for index, field in enumerate(self.state_list):
            if field == "throttle":
                val = self.throttle
            elif field == "aileron":
                val = self.aileron * self.qbar
            elif field == "abs(aileron)":
                val = abs(self.aileron) * self.qbar
            elif field == "elevator":
                val = self.elevator * self.qbar
            elif field == "rudder":
                val = self.rudder * self.qbar
            elif field == "abs(rudder)":
                val = abs(self.rudder) * self.qbar
            elif field == "flaps":
                val = self.flaps * self.qbar
            elif field == "motor[0]":
                val = self.motors[0]
            elif field == "motor[1]":
                val = self.motors[1]
            elif field == "motor[2]":
                val = self.motors[2]
            elif field == "motor[3]":
                val = self.motors[3]
            elif field == "motor[4]":
                val = self.motors[4]
            elif field == "motor[5]":
                val = self.motors[5]
            elif field == "lift":
                val = self.lift
            elif field == "drag":
                val = self.drag
            elif field == "thrust":
                val = self.thrust
            elif field == "bgx":
                val = self.g_body[0]
            elif field == "bgy":
                val = self.g_body[1]
            elif field == "abs(bgy)":
                val = abs(self.g_body[1])
            elif field == "abs(bgy)":
                val = abs(self.g_body[1])
            elif field == "bgz":
                val = self.g_body[2]
            elif field == "bax":
                val = self.a_body[0]
            elif field == "bay":
                val = self.a_body[1]
            elif field == "abs(bay)":
                val = abs(self.a_body[1])
            elif field == "baz":
                val = self.a_body[2]
            elif field == "qbar":
                val = self.qbar
            elif field == "alpha":
                val = sin(self.alpha) * self.qbar
            elif field == "alpha_prev1":
                val = sin(self.alpha_prev1) * self.qbar
            elif field == "alpha_prev2":
                val = sin(self.alpha_prev2) * self.qbar
            elif field == "beta":
                val = sin(self.beta) * self.qbar
            elif field == "beta_prev1":
                val = sin(self.beta_prev1) * self.qbar
            elif field == "beta_prev2":
                val = sin(self.beta_prev2) * self.qbar
            elif field == "p":
                val = self.gyros[0]
            elif field == "q":
                val = self.gyros[1]
            elif field == "r":
                val = self.gyros[2]
            elif field == "p_prev1":
                val = self.gyros_prev1[0]
            elif field == "q_prev1":
                val = self.gyros_prev1[1]
            elif field == "r_prev1":
                val = self.gyros_prev1[2]
            elif field == "p_prev2":
                val = self.gyros_prev2[0]
            elif field == "q_prev2":
                val = self.gyros_prev2[1]
            elif field == "r_prev2":
                val = self.gyros_prev2[2]
            elif field == "ax":
                val = self.accels[0]
            elif field == "ay":
                val = self.accels[1]
            elif field == "abs(ay)":
                val = abs(self.accels[1])
            elif field == "az":
                val = self.accels[2]
            elif field == "ax_prev1":
                val = self.accels_prev1[0]
            elif field == "ay_prev1":
                val = self.accels_prev1[1]
            elif field == "az_prev1":
                val = self.accels_prev1[2]
            elif field == "K":
                val = 1.0
            else:
                raise Exception("Sorry, unknown field name requested:", field)
            #if True and params is not None and field in self.output_states:
            if params is not None:
                param = params[index]
                if param["type"] == "output":
                    min = param["min"]
                    max = param["max"]
                    std = param["std"]
                    n = 2
                    if val < min - n*std:
                        val = min - n*std
                        print(field, "clipped to:", val)
                    if val > max + n*std:
                        val = max + n*std
                        print(field, "clipped to:", val)
            result.append(val)
        return result

    def state2dict(self, state):
        result = {}
        for i in range(len(state)):
            result[self.state_list[i]] = state[i]
        return result

    def output2dict(self, state):
        output_list = self.get_state_index( self.output_states )
        result = {}
        for i in range(len(output_list)):
            result[self.state_list[output_list[i]]] = state[i]
        return result

