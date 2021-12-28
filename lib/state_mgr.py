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
        self.qbar = 0
        self.alpha = 0
        self.beta = 0
        self.flying = False
        self.g_ned = np.array( [0.0, 0.0, gravity] )
        self.g_body = np.array( [0.0, 0.0, 0.0] )
        self.v_ned = np.array( [0.0, 0.0, 0.0] )
        self.v_body = np.array( [0.0, 0.0, 0.0] )

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

    def set_ned_velocity(self, vn, ve, vd):
        self.v_ned = np.array( [vn, ve, vd] )

    def set_body_velocity(self, vx, vy, vz):
        self.v_body = np.array( [vx, vy, vz] )

    def is_flying(self):
        # ground speed mps (ned)
        gs_mps = sqrt( self.v_ned[0]**2 + self.v_ned[1]**2 )

        # test if we are flying?
        if not self.flying and gs_mps > 10 and self.airspeed_mps > 7:
            print("Start flying @ %.2f" % self.time)
            self.flying = True
        elif self.flying and gs_mps < 5 and self.airspeed_mps < 3:
            print("Stop flying @ %.2f" % self.time)
            self.flying = False
        return self.flying

    def compute_body_frame_values(self, body_vel=True):
        if self.dt is None:
            print("Did you forget to set dt for this system?")
            return None

        # transformation between NED coordations and body coordinates
        ned2body = quaternion.eul2quat( self.phi_rad, self.the_rad,
                                        self.psi_rad )

        # rotate ned gravity vector into body frame
        self.g_body = quaternion.transform(ned2body, self.g_ned)
        
        if body_vel:
            # compute ned velocity with wind vector removed
            v_ned = np.array( [self.v_ned[0] + self.wn_filt, self.v_ned[1] + self.we_filt,
                               self.v_ned[2]] )
            #print(self.psi_rad*r2d, [self.wn_filt, self.we_filt], [self.v_ned[0], self.v_ned[1], self.v_ned[2]], v_ned)

            # rotate ned velocity vector into body frame
            self.v_body = quaternion.transform(ned2body, v_ned)
            #print(" ", self.v_body)

        # compute alpha and beta from body frame velocity
        self.alpha = atan2( self.v_body[2], self.v_body[0] )
        self.beta = atan2( -self.v_body[1], self.v_body[0] )
        #print("v(body):", v_body, "alpha = %.1f" % (self.alpha/d2r), "beta = %.1f" % (self.beta/d2r))

    def gen_state_vector(self):
        result = []
        for field in self.state_list:
            if field == "throttle":
                result.append( self.throttle )
            elif field == "sqrt(throttle)":
                result.append( sqrt(self.throttle) )
            elif field == "aileron":
                result.append( self.aileron * self.qbar )
            elif field == "abs(aileron)":
                result.append( abs(self.aileron * self.qbar) )
            elif field == "elevator":
                result.append( self.elevator * self.qbar )
            elif field == "rudder":
                result.append( self.rudder * self.qbar )
            elif field == "abs(rudder)":
                result.append( abs(self.rudder * self.qbar) )
            elif field == "bgx":
                result.append( self.g_body[0] )
            elif field == "bgy":
                result.append( self.g_body[1] )
            elif field == "bgz":
                result.append( self.g_body[2] )
            elif field == "bvx":
                result.append( self.v_body[0] * self.qbar )
            elif field == "bvy":
                result.append( self.v_body[1] * self.qbar )
            elif field == "bvz":
                result.append( self.v_body[2] * self.qbar )
            elif field == "p":
                result.append( self.p )
            elif field == "q":
                result.append( self.q )
            elif field == "r":
                result.append( self.r )
            else:
                print("Unknown field requested:", field, "aborting ...")
                quit()
        return result
    
    def state2dict(self, state):
        result = {}
        for i in range(len(state)):
            result[self.state_list[i]] = state[i]
        return result
        
