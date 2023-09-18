"""sim

Uses the system model derived by the system_id module to perform a
flight simulation.

Author: Curtis L. Olson, University of Minnesota, Dept of Aerospace
Engineering and Mechanics, UAV Lab.

"""

import json
from math import asin, atan2, cos, sin, pi
from matplotlib import pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from lib import quaternion
from lib.constants import d2r, r2d, gravity
from lib.state_mgr import StateManager

class Simulator():
    def __init__(self):
        self.A = None
        self.dt = None
        self.trand = None
        self.state_mgr = StateManager()
        self.reset()

    def load(self, model):
        f = open(model, "r")
        model = json.load(f)
        print(model)
        f.close()
        self.params = model["parameters"]
        cols = len(self.params)
        rows = 0
        for param in self.params:
            if param["type"] == "output":
                rows += 1
        print("size:", rows, "x", cols)
        self.dt = model["dt"]
        self.A = np.array(model["A"]).reshape(rows, cols)
        print("A:\n", self.A)
        internal_states = []
        output_states = []
        for param in model["parameters"]:
            if param["type"] == "internal":
                internal_states.append( param["name"] )
            else:
                output_states.append( param["name"] )
        self.state_mgr.set_state_names( internal_states, output_states )
        self.state_mgr.set_dt( self.dt )

    def reset(self):
        initial_airspeed_mps = 50.0
        self.state_mgr.set_airdata( initial_airspeed_mps, 0, 0 )
        self.state_mgr.set_throttle( 0.5 )
        self.state_mgr.set_flight_surfaces( aileron=0.0,
                                            elevator=-0.1,
                                            rudder=0.0 )
        self.pos_ned = np.array( [0.0, 0.0, 0.0] )
        self.vel_ned = np.array( [initial_airspeed_mps, 0.0, 0.0] )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2],
                                         0.0, 0.0, 0.0 )
        # self.state_mgr.set_body_velocity( initial_airspeed_mps, 0.0, 0.0 )
        self.state_mgr.set_orientation( 0, 0, 0 )
        self.state_mgr.ned2body = quaternion.eul2quat( 0, 0, 0 )
        self.state_mgr.set_gyros( np.array([0.0, 0.0, 0.0]) )
        self.state_mgr.set_accels( np.array([0.0, 0.0, 0.0]) )
        self.time = 0.0
        self.state_mgr.set_time( self.time )
        #self.last_vel_body = None
        self.data = []

    def trim_error(self, xk):
        self.state_mgr.set_throttle( xk[0] )
        self.state_mgr.set_flight_surfaces(xk[1], xk[2], xk[3])
        self.state_mgr.set_orientation(xk[4], xk[5], 0)
        self.state_mgr.alpha = xk[5]
        self.state_mgr.set_airdata(self.trim_airspeed_mps)
        self.state_mgr.set_wind(0.0, 0.0)
        self.state_mgr.set_gyros(0.0, 0.0, 0.0)
        self.state_mgr.set_ned_velocity(self.trim_airspeed_mps, 0.0, 0.0,
                                        0.0, 0.0, 0.0)
        self.state_mgr.compute_body_frame_values(compute_body_vel=False)
        state = self.state_mgr.gen_state_vector(self.params)
        next = self.A @ state
        current = self.state_mgr.state2dict(state)
        result = self.state_mgr.state2dict(next)
        print(result)
        errors = []
        next_asi = result["airspeed"]
        if next_asi < 0: next_asi = 0
        errors.append(self.trim_airspeed_mps - next_asi)
        #errors.append(result["lift"] - result["bgz"])
        errors.append(result["thrust"] - result["drag"])
        #errors.append(result["bax"])
        errors.append(result["bay"])
        #errors.append(result["baz"])
        errors.append(result["p"])
        errors.append(result["q"])
        errors.append(result["r"])
        print(errors)
        return errors

    def trim(self, airspeed_mps):
        self.trim_airspeed_mps = airspeed_mps
        initial = [0.5, 0.0, 0.0, 0.0, 0.0, 0.08]
        res = least_squares(self.trim_error, initial, verbose=2)
        print("res:", res)
        print("throttle:", res["x"][0])
        print("aileron:", res["x"][1])
        print("elevator:", res["x"][2])
        print("rudder:", res["x"][3])
        print("phi:", res["x"][3])
        print("theta:", res["x"][4])

    def add_noise(self, next):
        #print(self.trand)
        for i in range(len(next)):
            if "noise" in self.params[i]:
                sum = 0
                if self.trand is None:
                    self.trand = np.random.rand(len(self.params[i]["noise"])) * 2 * pi
                for j, pt in enumerate(self.params[i]["noise"]):
                    sum += sin(self.trand[j]+self.time*2*pi*pt[0]) * pt[1]
                    #print(i, sum)
                next[i] += sum

    def update(self):
        state = self.state_mgr.gen_state_vector(self.params)
        # print("state->", self.state_mgr.state2dict(state))

        next = self.A @ state
        self.add_noise(next)

        input = self.state_mgr.state2dict(state)
        result = self.state_mgr.output2dict(next)
        #print("state:", state)
        #print("next:", result)
        #print()

        # rotational rates: predicted directly from the state update
        p = result["p"]
        q = result["q"]
        r = result["r"]
        self.state_mgr.set_gyros( np.array([p, q, r]) )

        # integrate body rates
        self.state_mgr.update_attitude()

        # gravity in body frame
        self.state_mgr.update_gravity_body()

        ax = result["ax"]  # sum of thrust - drag
        ay = result["ay"]
        az = result["az"]
        self.state_mgr.set_accels( np.array([ax, ay, az]))

        # alpha/beta are modeled as sin(angle) * qbar
        if self.state_mgr.qbar > 0.1:
            sa = result["alpha"] / self.state_mgr.qbar
            sb = result["beta"] / self.state_mgr.qbar
            if sa < -0.25*pi: sa = -0.25*pi
            if sa >  0.25*pi: sa =  0.25*pi
            if sb < -0.25*pi: sb = -0.25*pi
            if sb >  0.25*pi: sb =  0.25*pi
            alpha_rad = asin(sa)
            beta_rad = asin(sb)
        else:
            alpha_rad = 0
            beta_rad = 0

        self.state_mgr.update_airdata(alpha_rad, beta_rad)

        # velocity in ned frame
        self.vel_ned = quaternion.backTransform( self.state_mgr.ned2body, self.state_mgr.v_body )
        self.state_mgr.set_ned_velocity( self.vel_ned[0], self.vel_ned[1], self.vel_ned[2], 0.0, 0.0, 0.0 )

        # update position
        self.pos_ned += self.vel_ned * self.dt

        # store data point
        self.data.append(
            [ self.time, self.state_mgr.airspeed_mps,
              self.state_mgr.throttle,
              self.state_mgr.aileron,
              self.state_mgr.elevator,
              self.state_mgr.rudder,
              self.state_mgr.phi_rad, self.state_mgr.the_rad, self.state_mgr.psi_rad,
              self.state_mgr.alpha, self.state_mgr.beta,
              self.state_mgr.gyros[0], self.state_mgr.gyros[1], self.state_mgr.gyros[2]] )
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
        plt.plot( self.data[:,0], self.data[:,11]*r2d, label="Roll rate (deg/sec)" )
        plt.plot( self.data[:,0], self.data[:,12]*r2d, label="Pitch rate (deg/sec)" )
        plt.plot( self.data[:,0], self.data[:,13]*r2d, label="Yaw rate (deg/sec)" )
        plt.legend()
        plt.figure()
        plt.plot( self.data[:,0], self.data[:,19], label="Pos 'down' (m)" )
        plt.legend()
        plt.show()
