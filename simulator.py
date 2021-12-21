#!/usr/bin/env python3

"""sim

Uses the system model derived by the system_id module to perform a
flight simulation.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import json
import math
from matplotlib import pyplot as plt
import numpy as np

import quaternion
from constants import r2d
from state_mgr import StateManager

class Simulator():
    def __init__(self):
        self.A = None
        self.dt = 0.01
        self.state_mgr = StateManager()
        self.reset()

    def load(self, model):
        f = open(model, "r")
        model = json.load(f)
        print(model)
        f.close()
        self.dt = model["dt"]
        self.A = np.array(model["A"])
        #print(self.A)
        state_names = []
        for param in model["parameters"]:
            state_names.append( param["name"] )
        self.state_mgr.set_state_names( state_names )
        self.state_mgr.set_dt( self.dt )

    def reset(self):
        initial_airspeed_mps = 15.0
        self.state_mgr.set_airdata( initial_airspeed_mps )
        self.state_mgr.set_throttle( 0.5 )
        self.state_mgr.set_flight_surfaces( aileron=0.0,
                                            elevator=-0.05,
                                            rudder=0.0 )
        self.alpha = 0.0
        self.beta = 0.0
        self.pos_ned = np.array( [0.0, 0.0, 0.0] )
        self.vel_ned = np.array( [initial_airspeed_mps, 0.0, 0.0] )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2] )
        self.phi_rad = 0.0
        self.the_rad = 0.0
        self.psi_rad = 0.0
        self.state_mgr.set_orientation( self.phi_rad,
                                        self.the_rad,
                                        self.psi_rad )
        self.ned2body = quaternion.eul2quat( self.phi_rad,
                                             self.the_rad,
                                             self.psi_rad )
        self.bax = 0.0
        self.bay = 0.0
        self.baz = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.state_mgr.set_gyros( self.p, self.q, self.r )
        self.time = 0.0
        self.state_mgr.set_time( self.time )
        #self.last_vel_body = None
        self.data = []

    def state2dict(self, state):
        result = {}
        for i in range(len(state)):
            result[self.state_mgr.state_list[i]] = state[i]
        return result
        
    def update(self):
        state = self.state_mgr.gen_state_vector(flight_check=False)
        #print(self.state2dict(state))
        # state = np.array( [ self.airspeed_mps**2,
        #                     self.throttle,
        #                     self.aileron, abs(self.aileron),
        #                     self.elevator,
        #                     self.rudder, abs(self.rudder),
        #                     self.phi_rad, math.cos(self.phi_rad),
        #                     self.the_rad,
        #                     self.alpha,
        #                     self.beta, math.cos(self.beta),
        #                     self.bax, self.bay, self.baz,
        #                     self.p, self.q, self.r ] )

        next = self.A @ state

        input = self.state2dict(state)
        result = self.state2dict(next)
        #print("state:", state)
        #print("next:", next)
        #print()

        if result["airspeed**2"] > 0:
            self.airspeed_mps = math.sqrt(result["airspeed**2"])
        else:
            self.airspeed_mps = 0
        self.alpha = result["alpha"]
        self.beta = result["beta"]
        self.state_mgr.set_airdata(self.airspeed_mps, self.alpha, self.beta)
        #self.bax = next[13]
        #self.bay = next[14]
        #self.baz = next[15]
        #self.p = result["p"]
        self.q = result["q"]
        self.r = result["r"]
        self.state_mgr.set_gyros(self.p, self.q, self.r)

        # update attitude
        rot_body = quaternion.eul2quat(self.p * self.dt,
                                       self.q * self.dt,
                                       self.r * self.dt)
        self.ned2body = quaternion.multiply(self.ned2body, rot_body)
        #self.phi_rad, self.the_rad, self.psi_rad = quat2eul(self.ned2body)
        lock_phi_rad, self.the_rad, self.psi_rad = \
            quaternion.quat2eul(self.ned2body)
        self.state_mgr.set_orientation(lock_phi_rad, self.the_rad, self.psi_rad)
        
        # velocity in body frame
        bd = math.sin(self.alpha) * self.airspeed_mps
        be = -math.sin(self.beta) * self.airspeed_mps
        bn2 = self.airspeed_mps**2 - bd**2 - be**2
        if bn2 > 0:
            bn = math.sqrt( bn2 )
        else:
            bn = 0

        # accel in body frame
        # vel_body = np.array( [bn, be, bd] )
        # if self.last_vel_body is None:
        #     self.last_vel_body = vel_body.copy()
        # accel_body = vel_body - self.last_vel_body
        # self.last_vel_body = vel_body.copy()
        self.bax = input["accel_body[0]"]
        self.bay = input["accel_body[1]"]
        self.baz = input["accel_body[2]"]
        # velocity in ned fram
        self.vel_ned = quaternion.backTransform( self.ned2body,
                                                 np.array([bn, be, bd]) )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2] )

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
              self.alpha, self.beta,
              self.bax, self.bay, self.baz,
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
        plt.plot( self.data[:,0], self.data[:,15]*r2d, label="Pitch rate (deg/sec)" )
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
