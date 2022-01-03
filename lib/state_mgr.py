# manage construction of the state vector by name.  Intended to help
# avoid getting lost in our index offsets as we play around with
# different state parameters.

from math import atan2, cos, sin, sqrt
import numpy as np

from lib.constants import d2r, r2d, gravity, kt2mps
from lib import quaternion

class StateManager():
    def __init__(self):
        self.ind_states = []
        self.dep_states = []
        self.state_list = []
        self.dt = None
        self.wn_filt = 0
        self.we_filt = 0
        self.wd_filt = 0
        self.qbar = 0
        self.lift = 0
        self.Cl = 0
        self.drag = 0
        self.Cd = 0
        self.thrust = 0
        self.alpha = 0
        self.beta = 0
        self.flying = False
        self.g_ned = np.array( [0.0, 0.0, gravity] )
        self.g_body = np.array( [0.0, 0.0, 0.0] )
        self.g_flow = np.array( [0.0, 0.0, 0.0] )
        self.v_ned = np.array( [0.0, 0.0, 0.0] )
        self.v_body = np.array( [0.0, 0.0, 0.0] )
        self.v_body_last = None
        self.v_flow = np.array( [0.0, 0.0, 0.0] )
        self.v_flow_last = None
        self.a_body = np.array( [0.0, 0.0, 0.0] )
        self.a_flow = np.array( [0.0, 0.0, 0.0] )
        self.update_count = 0
        self.last_update_count = 0

    def set_state_names(self, ind_states, dep_states):
        self.ind_states = ind_states
        self.dep_states = dep_states
        self.state_list = self.ind_states + self.dep_states

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

    def set_time(self, time):
        self.time = time
        
    def set_throttle(self, throttle):
        self.throttle = throttle
        if self.throttle < 0: self.throttle = 0
        if self.throttle > 1: self.throttle = 1
        # max thrust is 0.75 gravity, so we can't quite hover on full power
        self.thrust = sqrt(self.throttle) * 0.75 * abs(gravity)
        
    def set_flight_surfaces(self, aileron, elevator, rudder):
        self.aileron = aileron
        self.elevator = elevator
        self.rudder = rudder
        if self.aileron < -1: self.aileron = -1
        if self.aileron > 1: self.aileron = 1
        if self.elevator < -1: self.elevator = -1
        if self.elevator > 1: self.elevator = 1
        if self.rudder < -1: self.rudder = -1
        if self.rudder > 1: self.rudder = 1

    def set_orientation(self, phi_rad, the_rad, psi_rad):
        self.phi_rad = phi_rad
        self.the_rad = the_rad
        self.psi_rad = psi_rad

    def set_airdata(self, airspeed_mps):
        self.airspeed_mps = airspeed_mps
        self.qbar = 0.5 * self.airspeed_mps**2

    def set_wind(self, wn, we):
        self.wn_filt = 0.95 * self.wn_filt + 0.05 * wn
        self.we_filt = 0.95 * self.we_filt + 0.05 * we

    def set_gyros(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r

    def set_ned_velocity(self, vn, ve, vd, wn, we, wd):
        # store NED velocity, corrected to remove wind effects
        self.wn_filt = 0.95 * self.wn_filt + 0.05 * wn
        self.we_filt = 0.95 * self.we_filt + 0.05 * we
        self.wd_filt = 0.95 * self.wd_filt + 0.05 * wd
        self.v_ned = np.array( [vn + wn, ve + we, vd + wd] )

    def set_body_velocity(self, vx, vy, vz):
        self.v_body = np.array( [vx, vy, vz] )

    def is_flying(self):
        # ground speed mps (ned)
        gs_mps = sqrt( self.v_ned[0]**2 + self.v_ned[1]**2 )

        # test if we are flying?
        if not self.flying and gs_mps > 10 and self.airspeed_mps > 7:
            print("Start flying @ %.2f" % self.time)
            self.flying = True
        elif self.flying and gs_mps < 5 and self.airspeed_mps < 5:
            print("Stop flying @ %.2f" % self.time)
            self.flying = False
        return self.flying

    # compute body (AND FLOW) frame of reference values
    def compute_body_frame_values(self, compute_body_vel=True):
        self.update_count += 1
        if self.dt is None:
            print("Did you forget to set dt for this system?")
            return None

        # transformation between NED coordations and body coordinates
        ned2body = quaternion.eul2quat( self.phi_rad, self.the_rad,
                                        self.psi_rad )

        if compute_body_vel:
            # rotate ned velocity vector into body frame
            self.v_body = quaternion.transform(ned2body, self.v_ned)
            #print("v_ned:", self.v_ned, np.linalg.norm(self.v_ned),
            #      "v_body:", self.v_body, np.linalg.norm(self.v_body))
            
            # compute alpha and beta from body frame velocity
            self.alpha = atan2( self.v_body[2], self.v_body[0] )
            self.beta = atan2( -self.v_body[1], self.v_body[0] )
            #print("v(body):", v_body, "alpha = %.1f" % (self.alpha/d2r), "beta = %.1f" % (self.beta/d2r))

        # compute flow frame of reference and velocity
        body2flow = quaternion.eul2quat(0.0, -self.alpha, -self.beta)
        ned2flow = quaternion.multiply(ned2body, body2flow)
        #fphi, fthe, fpsi = quaternion.quat2eul(ned2flow)
        #print("alpha: %.1f" % (self.alpha*r2d), "beta: %.1f" % (self.beta*r2d))
        #print(" body (deg): %.1f %.1f %.1f" % (self.phi_rad*r2d, self.the_rad*r2d, self.psi_rad*r2d))
        #print(" flow (deg): %.1f %.1f %.1f" % (fphi*r2d, fthe*r2d, fpsi*r2d))
        self.v_flow = quaternion.transform(ned2flow, self.v_ned)
        
        # compute acceleration in the body frame
        if self.v_body_last is None:
            self.v_body_last = self.v_body.copy()
        if not np.array_equal(self.v_body, self.v_body_last):
            dt = self.dt * (self.update_count - self.last_update_count)
            self.a_body = (self.v_body - self.v_body_last) / dt
            #print(self.update_count, "body:", self.v_body, self.v_body_last, self.a_body)
            self.v_body_last = self.v_body.copy()
            #print("a_body norm1: ", np.linalg.norm(self.a_body))
            
        # compute acceleration in the flow frame
        if self.v_flow_last is None:
            self.v_flow_last = self.v_flow.copy()
        if not np.array_equal(self.v_flow, self.v_flow_last):
            dt = self.dt * (self.update_count - self.last_update_count)
            self.a_flow = (self.v_flow - self.v_flow_last) / dt
            #print(self.update_count, "flow:", self.v_flow, self.v_flow_last, self.a_flow)
            self.v_flow_last = self.v_flow.copy()
            #print("a_flow norm1: ", np.linalg.norm(self.a_flow))
            self.last_update_count = self.update_count
            
        # lift, drag, and weight vector estimates

        # rotate ned gravity vector into body frame
        self.g_body = quaternion.transform(ned2body, self.g_ned)

        # rotate ned gravity vector into flow frame
        self.g_flow = quaternion.transform(ned2flow, self.g_ned)
        
        # is my math correct here? (for drag need grav & body_accel in
        # flight path frame of reference ... I think.
        
        self.drag = (self.thrust - self.g_flow[0]) - self.a_flow[0]
        if self.qbar > 10:
            self.Cd = self.drag / self.qbar
        else:
            self.Cd = 0

        # do I need to think through lift frame of reference here too?
        
        self.lift = -self.a_flow[2] - self.g_flow[2]
        if self.qbar > 10:
            self.Cl = self.lift / self.qbar
        else:
            self.Cl = 0
        #print(" lift:", self.a_flow[2], self.g_flow[2], self.lift)
        
    def gen_state_vector(self, params=None):
        result = []
        for index, field in enumerate(self.state_list):
            if field == "throttle":
                val = self.throttle
            elif field == "aileron":
                val = self.aileron * self.qbar
            elif field == "abs(aileron)":
                val = abs(self.aileron * self.qbar)
            elif field == "elevator":
                val = self.elevator * self.qbar
            elif field == "rudder":
                val = self.rudder * self.qbar
            elif field == "abs(rudder)":
                val = abs(self.rudder * self.qbar)
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
            elif field == "bgz":
                val = self.g_body[2]
            elif field == "fgx":
                val = self.g_flow[0]
            elif field == "fgy":
                val = self.g_flow[1]
            elif field == "fgz":
                val = self.g_flow[2]
            elif field == "bax":
                val = self.a_body[0]
            elif field == "bay":
                val = self.a_body[1]
            elif field == "baz":
                val = self.a_body[2]
            elif field == "fax":
                val = self.a_flow[0]
            elif field == "fay":
                val = self.a_flow[1]
            elif field == "faz":
                val = self.a_flow[2]
            elif field == "airspeed":
                val = self.airspeed_mps
            elif field == "sin(alpha)":
                val = sin(self.alpha) * self.qbar
            elif field == "sin(beta)":
                val = sin(self.beta) * self.qbar
            elif field == "abs(sin(beta))":
                val = abs(sin(self.beta)) * self.qbar
            elif field == "bvx":
                #val = self.v_body[0]**2 * 0.5 * np.sign(self.v_body[0])
                val = self.v_body[0]
            elif field == "bvy":
                #val = self.v_body[1]**2 * 0.5 * np.sign(self.v_body[1])
                val = self.v_body[1]
            elif field == "bvz":
                #val = self.v_body[2]**2 * 0.5 * np.sign(self.v_body[2])
                val = self.v_body[2]
            elif field == "p":
                val = self.p
            elif field == "q":
                val = self.q
            elif field == "r":
                val = self.r
            else:
                print("Unknown field requested:", field, "aborting ...")
                quit()
            if params is not None and field in self.dep_states:
                param = params[index]
                min = param["min"]
                max = param["max"]
                median = param["median"]
                std = param["std"]
                n = 5
                if val < median - n*std:
                    val = median - n*std
                    print(field, "clipped to:", val)
                if val > median + n*std:
                    val = median + n*std
                    print(field, "clipped to:", val)
            result.append(val)
        return result
    
    def state2dict(self, state):
        result = {}
        for i in range(len(state)):
            result[self.state_list[i]] = state[i]
        return result
        
