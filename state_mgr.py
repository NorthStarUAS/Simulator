# manage construction of the state vector by name.  Intended to help
# avoid getting lost in our index offsets as we play around with
# different state parameters.

import math
import numpy as np

from constants import d2r, kt2mps
import quaternion

class StateManager():
    def __init__(self):
        self.state_list = []
        self.dt = None
        self.vn = 0.0
        self.ve = 0.0
        self.vd = 0.0
        self.wn_filt = 0
        self.we_filt = 0
        self.alpha = 0
        self.beta = 0
        self.compute_alpha_beta = True
        self.flying = False
        self.v_body_last = None

    def set_state_names(self, state_list):
        self.state_list = state_list

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
        
    def set_flight_surfaces(self, aileron, elevator, rudder):
        self.aileron = aileron
        self.elevator = elevator
        self.rudder = rudder

    def set_orientation(self, phi_rad, the_rad, psi_rad):
        self.phi_rad = phi_rad
        self.the_rad = the_rad
        self.psi_rad = psi_rad

    def set_airdata(self, airspeed_mps, alpha=None, beta=None):
        self.airspeed_mps = airspeed_mps
        if alpha is not None:
            self.alpha = alpha
            self.beta = beta
            self.compute_alpha_beta = False

    def set_wind(self, wn, we):
        self.wn_filt = 0.95 * self.wn_filt + 0.05 * wn
        self.we_filt = 0.95 * self.we_filt + 0.05 * we

    def set_gyros(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r

    def set_ned_velocity(self, vn, ve, vd):
        self.vn = vn
        self.ve = ve
        self.vd = vd
        
    def gen_state_vector(self, flight_check=True):
        if self.dt is None:
            print("Did you forget to set dt for this system?")
            return None
        
        # ground speed mps (ned)
        gs_mps = math.sqrt( self.vn**2 + self.ve**2 )

        # test if we are flying?
        if flight_check:
            if not self.flying and gs_mps > 10 and self.airspeed_mps > 7:
                print("Start flying @ %.2f" % self.time)
                self.flying = True
            elif self.flying and gs_mps < 5 and self.airspeed_mps < 3:
                print("Stop flying @ %.2f" % self.time)
                self.flying = False
                self.v_body_last = None
            if not self.flying:
                return None
        else:
            self.flying = True
        
        # transformation between NED coordations and body coordinates
        ned2body = quaternion.eul2quat( self.phi_rad, self.the_rad,
                                        self.psi_rad )

        # compute ned velocity with wind vector removed
        v_ned = np.array( [self.vn + self.wn_filt, self.ve + self.we_filt,
                           self.vd] )
        #print(v_ned)

        # rotate ned velocity vector into body frame
        v_body = quaternion.transform(ned2body, v_ned)

        if self.compute_alpha_beta:
            # estimate alpha and beta (requires a decent wind estimate)
            self.alpha = math.atan2( v_body[2], v_body[0] )
            self.beta = math.atan2( -v_body[1], v_body[0] )
            #print("v(body):", v_body, "alpha = %.1f" % (self.alpha/d2r), "beta = %.1f" % (self.beta/d2r))

        # estimate accelerations in body frame using velocity difference
        # (imu accel biases are too problematic)
        if self.v_body_last is None:
            self.v_body_last = v_body.copy()
        accel_body = (v_body - self.v_body_last) / self.dt
        self.v_body_last = v_body.copy()

        result = []
        for field in self.state_list:
            if field == "airspeed**2":
                result.append( self.airspeed_mps**2 )
            elif field == "throttle":
                result.append( self.throttle )
            elif field == "aileron":
                result.append( self.aileron )
            elif field == "abs(aileron)":
                result.append( abs(self.aileron) )
            elif field == "elevator":
                result.append( self.elevator )
            elif field == "rudder":
                result.append( self.rudder )
            elif field == "abs(rudder)":
                result.append( abs(self.rudder) )
            elif field == "phi":
                result.append( self.phi_rad )
            elif field == "abs(phi)":
                result.append( abs(self.phi_rad) )
            elif field == "cos(phi)":
                result.append( math.cos(self.phi_rad) )
            elif field == "the":
                result.append( self.the_rad )
            elif field == "vd":
                result.append( self.vd )
            elif field == "alpha":
                result.append( self.alpha )
            elif field == "sin(alpha)":
                result.append( math.sin(self.alpha) )
            elif field == "beta":
                result.append( self.beta )
            elif field == "abs(beta)":
                result.append( abs(self.beta) )
            elif field == "cos(beta)":
                result.append( math.cos(self.beta) )
            elif field == "accel_body[0]":
                result.append( accel_body[0] )
            elif field == "accel_body[1]":
                result.append( accel_body[1] )
            elif field == "accel_body[2]":
                result.append( accel_body[2] )
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
