"""sim

Uses the system model derived by the system_id module to perform a
flight simulation.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import json
from math import asin, cos, sin, sqrt
from matplotlib import pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from lib import quaternion
from lib.constants import d2r, r2d
from lib.state_mgr import StateManager

class Simulator():
    def __init__(self):
        self.A = None
        self.dt = None
        self.state_mgr = StateManager()
        self.reset()

    def load(self, model):
        f = open(model, "r")
        model = json.load(f)
        print(model)
        f.close()
        self.dt = model["dt"]
        self.A = np.array(model["A"])
        print(self.A)
        ind_states = []
        dep_states = []
        for param in model["parameters"]:
            if param["type"] == "independent":
                ind_states.append( param["name"] )
            else:
                dep_states.append( param["name"] )
        self.state_mgr.set_state_names( ind_states, dep_states )
        self.state_mgr.set_dt( self.dt )

    def reset(self):
        initial_airspeed_mps = 10.0
        self.state_mgr.set_airdata( initial_airspeed_mps )
        self.state_mgr.set_throttle( 0.5 )
        self.state_mgr.set_flight_surfaces( aileron=0.0,
                                            elevator=-0.1,
                                            rudder=0.0 )
        self.airspeed_mps = 0
        self.pos_ned = np.array( [0.0, 0.0, 0.0] )
        self.vel_ned = np.array( [initial_airspeed_mps, 0.0, 0.0] )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2],
                                         0.0, 0.0, 0.0 )
        self.state_mgr.set_body_velocity( initial_airspeed_mps, 0.0, 0.0 )
        self.phi_rad = 0.0
        self.the_rad = 0.0
        self.psi_rad = 0.0
        self.state_mgr.set_orientation( self.phi_rad,
                                        self.the_rad,
                                        self.psi_rad )
        self.ned2body = quaternion.eul2quat( self.phi_rad,
                                             self.the_rad,
                                             self.psi_rad )
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.state_mgr.set_gyros( self.p, self.q, self.r )
        self.time = 0.0
        self.state_mgr.set_time( self.time )
        #self.last_vel_body = None
        self.data = []

    def trim_error(self, xk):
        self.state_mgr.set_throttle( xk[0] )
        self.state_mgr.set_flight_surfaces(xk[1], xk[2], 0)
        self.state_mgr.set_orientation(xk[3], xk[4], 0)
        self.state_mgr.set_airdata(self.trim_airspeed_mps)
        self.state_mgr.set_wind(0.0, 0.0)
        self.state_mgr.set_gyros(0.0, 0.0, 0.0)
        self.state_mgr.set_ned_velocity(self.trim_airspeed_mps, 0.0, 0.0)
        self.state_mgr.compute_body_frame_values(body_vel=False)
        state = self.state_mgr.gen_state_vector()
        next = self.A @ state
        current = self.state_mgr.state2dict(state)
        result = self.state_mgr.state2dict(next)
        errors = []
        next_asi = result["qbar"]
        if next_asi < 0: next_asi = 0
        errors.append(self.trim_airspeed_mps - sqrt(next_asi))
        #errors.append(xk[4] - self.state_mgr.alpha)
        #errors.append(current["bvx"] - result["bvx"])
        #errors.append(current["bvy"] - result["bvy"])
        #errors.append(current["bvz"] - result["bvz"])
        errors.append(result["p"])
        errors.append(result["q"])
        errors.append(result["r"])
        print(errors)
        return errors
    
    def trim(self, airspeed_mps):
        self.trim_airspeed_mps = airspeed_mps
        initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        res = least_squares(self.trim_error, initial, verbose=2)
        print("res:", res)
        print("throttle:", res["x"][0])
        print("aileron:", res["x"][1])
        print("elevator:", res["x"][2])
        print("phi:", res["x"][3])
        print("theta:", res["x"][4])
        print("alpha:", res["x"][5])
        print("beta:", res["x"][6])

    def update(self):
        state = self.state_mgr.gen_state_vector()
        #print(self.state2dict(state))

        next = self.A @ state

        input = self.state_mgr.state2dict(state)
        result = self.state_mgr.state2dict(next)
        #print("state:", state)
        #print("next:", result)
        #print()

        if False:
            # debug
            idx_list = self.state_mgr.get_state_index( ["r"] )
            row = self.A[idx_list[0],:]
            print("r = ", end="")
            e = []
            for j in range(len(row)):
                e.append(state[j]*row[j])
            idx = np.argsort(-np.abs(e))
            for j in idx:
                print("%.3f (%s) " % (e[j], self.state_mgr.state_list[j]), end="")
            print("")

        last_qbar = self.state_mgr.qbar
        self.airspeed_mps = result["airspeed"]
        s_alpha = result["sin(alpha)"] / last_qbar
        s_beta = result["sin(beta)"] / last_qbar
        
        # protect against our linear state transtion going out of domain bounds
        if s_alpha > 1: s_alpha = 1
        if s_alpha < -1: s_alpha = -1
        if s_beta > 1: s_beta = 1
        if s_beta < -1: s_beta = -1
        print(s_alpha, s_beta)
        self.state_mgr.alpha = asin(s_alpha)
        self.state_mgr.beta = asin(s_beta)
        
        # protect against alpha/beta exceeding plausible thresholds
        # for normal flight conditions
        max_angle = 15 * d2r
        if self.state_mgr.alpha > max_angle: self.state_mgr.alpha = max_angle
        if self.state_mgr.alpha < -max_angle: self.state_mgr.alpha = -max_angle
        if self.state_mgr.beta > max_angle: self.state_mgr.beta = max_angle
        if self.state_mgr.beta < -max_angle: self.state_mgr.beta = -max_angle
        self.bvx = cos(self.state_mgr.alpha) * self.airspeed_mps
        self.bvy = sin(self.state_mgr.beta) * self.airspeed_mps
        self.bvz = sin(self.state_mgr.alpha) * self.airspeed_mps
        self.state_mgr.set_airdata(self.airspeed_mps)
        self.state_mgr.set_body_velocity( self.bvx, self.bvy, self.bvz )
        
        self.p = result["p"]
        self.q = result["q"]
        self.r = result["r"]
        self.state_mgr.set_gyros(self.p, self.q, self.r)

        # update attitude
        rot_body = quaternion.eul2quat(self.p * self.dt,
                                       self.q * self.dt,
                                       self.r * self.dt)
        self.ned2body = quaternion.multiply(self.ned2body, rot_body)
        self.phi_rad, self.the_rad, self.psi_rad = quaternion.quat2eul(self.ned2body)
        self.state_mgr.set_orientation(self.phi_rad, self.the_rad, self.psi_rad)

        self.state_mgr.compute_body_frame_values(compute_body_vel=False)
        
        # velocity in ned frame
        self.vel_ned = quaternion.backTransform( self.ned2body,
                                                 np.array([self.bvx, self.bvy, self.bvz]) )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2],
                                         0.0, 0.0, 0.0 )

        # update position
        self.pos_ned += self.vel_ned * self.dt

        # store data point
        self.data.append(
            [ self.time, self.airspeed_mps,
              self.state_mgr.throttle,
              self.state_mgr.aileron,
              self.state_mgr.elevator,
              self.state_mgr.rudder,
              self.phi_rad, self.the_rad, self.psi_rad,
              self.state_mgr.alpha, self.state_mgr.beta,
              self.p, self.q, self.r] )
        self.data[-1].extend( self.pos_ned.tolist() )
        self.data[-1].extend( self.vel_ned.tolist() )

        # update time
        self.time += self.dt

    def plot(self):
        self.data = np.array(self.data)
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,1], label="Airspeed (mps)" )
        plt.legend()
        plt.figure() 
        plt.plot( self.data[:,0], self.data[:,6]*r2d, label="Roll (deg)" )
        plt.plot( self.data[:,0], self.data[:,7]*r2d, label="Pitch (deg)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,9]*r2d, label="self.alpha (deg)" )
        plt.plot( self.data[:,0], self.data[:,10]*r2d, label="self.beta (deg)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,11], label="body ax (mps^2)" )
        plt.plot( self.data[:,0], self.data[:,12], label="body ay (mps^2)" )
        plt.plot( self.data[:,0], self.data[:,13], label="body az (mps^2)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,14]*r2d, label="Roll rate (deg/sec)" )
        plt.plot( self.data[:,0], self.data[:,15]*r2d, label="Pitch rate (deg/sec)" )
        plt.plot( self.data[:,0], self.data[:,16]*r2d, label="Yaw rate (deg/sec)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,19], label="Pos 'down' (m)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,20], label="Vel 'north' (m)" )
        plt.plot( self.data[:,0], self.data[:,21], label="Vel 'east' (m)" )
        plt.plot( self.data[:,0], self.data[:,22], label="Vel 'down' (m)" )
        plt.legend()
        plt.show()
