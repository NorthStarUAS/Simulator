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

class Simulator():
    def __init__(self):
        self.A = None
        self.dt = 0.01
        self.reset()

    def load(self, model):
        f = open(model, "r")
        model = json.load(f)
        print(model)
        f.close()
        self.dt = model["dt"]
        self.A = np.array(model["A"])
        #print(self.A)
        
    def reset(self):
        self.airspeed_mps = 17.3
        self.alpha = 0.07
        self.beta = 0.0
        self.pos_ned = np.array( [0.0, 0.0, 0.0] )
        self.vel_ned = np.array( [10.0, 0.0, 0.0] )
        self.phi_rad = 0.0
        self.the_rad = -0.04
        self.psi_rad = 0.0
        self.ned2body = quaternion.eul2quat(self.phi_rad,
                                            self.the_rad,
                                            self.psi_rad)
        self.bax = 0.0
        self.bay = 0.0
        self.baz = 0.0
        self.p = 0
        self.q = 0
        self.r = 0
        self.time = 0.0
        self.throttle = 0.5
        self.aileron = 0.0
        self.elevator = -0.05
        self.rudder = 0.0
        self.last_vel_body = None
        self.data = []

    def update(self):
        state = np.array( [ self.airspeed_mps**2,
                            self.throttle,
                            self.aileron, abs(self.aileron),
                            self.elevator,
                            self.rudder, abs(self.rudder),
                            self.phi_rad, math.cos(self.phi_rad),
                            self.the_rad,
                            self.alpha,
                            self.beta, math.cos(self.beta),
                            self.bax, self.bay, self.baz,
                            self.p, self.q, self.r ] )

        next = self.A @ state
        #print("state:", state)
        #print("next:", next)
        #print()

        if next[0] > 0:
            self.airspeed_mps = math.sqrt(next[0])
        else:
            self.airspeed_mps = 0
        self.alpha = next[10]
        self.beta = next[11]
        #self.bax = next[13]
        #self.bay = next[14]
        #self.baz = next[15]
        #self.p = next[16]
        self.q = next[17]
        self.r = next[18]

        # update attitude
        rot_body = quaternion.eul2quat(self.p * self.dt,
                                       self.q * self.dt,
                                       self.r * self.dt)
        self.ned2body = quaternion.multiply(self.ned2body, rot_body)
        #self.phi_rad, self.the_rad, self.psi_rad = quat2eul(self.ned2body)
        lock_phi_rad, self.the_rad, self.psi_rad = \
            quaternion.quat2eul(self.ned2body)

        # velocity in body frame
        bd = math.sin(self.alpha) * self.airspeed_mps
        be = -math.sin(self.beta) * self.airspeed_mps
        bn2 = self.airspeed_mps**2 - bd**2 - be**2
        if bn2 > 0:
            bn = math.sqrt( bn2 )
        else:
            bn = 0

        # accel in body frame
        vel_body = np.array( [bn, be, bd] )
        if self.last_vel_body is None:
            self.last_vel_body = vel_body.copy()
        accel_body = vel_body - self.last_vel_body
        self.last_vel_body = vel_body.copy()
        self.bax = accel_body[0]
        self.bay = accel_body[1]
        self.baz = accel_body[2]

        # velocity in ned fram
        self.vel_ned = quaternion.backTransform( self.ned2body,
                                                 np.array([bn, be, bd]) )

        # update position
        self.pos_ned += self.vel_ned * self.dt

        # store data point
        self.data.append(
            [ self.time, self.airspeed_mps,
              self.throttle, self.aileron, self.elevator, self.rudder,
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
        print(self.data[:,0])
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
        plt.plot( self.data[:,0], self.data[:,11]*r2d, label="body ax (mps^2)" )
        plt.plot( self.data[:,0], self.data[:,12]*r2d, label="body ay (mps^2)" )
        plt.plot( self.data[:,0], self.data[:,13]*r2d, label="body az (mps^2)" )
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
