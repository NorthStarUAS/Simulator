#!/usr/bin/env python3

import json
import math
from matplotlib import pyplot as plt
import numpy as np

d2r = math.pi / 180
r2d = 180 / math.pi
kt2mps = 0.5144444444444444444
mps2kt = 1.0 / kt2mps

f = open("skywalker_model.json", "r")
A_list = json.load(f)
f.close()
A = np.array(A_list)
print(A)

# computes a quaternion from the given euler angles
def eul2quat(phi_rad, the_rad, psi_rad):
    sin_psi = math.sin(psi_rad * 0.5)
    cos_psi = math.cos(psi_rad * 0.5)
    sin_the = math.sin(the_rad * 0.5)
    cos_the = math.cos(the_rad * 0.5)
    sin_phi = math.sin(phi_rad * 0.5)
    cos_phi = math.cos(phi_rad * 0.5)

    q = np.zeros(4)
    q[0] = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi  
    q[1] = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi
    q[2] = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi  
    q[3] = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi
    
    return q

# quaternion to euler angle: returns phi, the, psi
def quat2eul(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    m11 = 2*(q0*q0 + q1*q1) - 1
    m12 = 2*(q1*q2 + q0*q3)
    m13 = 2*(q1*q3 - q0*q2)
    m23 = 2*(q2*q3 + q0*q1)
    m33 = 2*(q0*q0 + q3*q3) - 1
    
    psi_rad = math.atan2(m12, m11)
    the_rad = math.asin(-m13)
    phi_rad = math.atan2(m23, m33)
    
    return phi_rad, the_rad, psi_rad

def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions.

    >>> q = quaternion_multiply([4, 1, -2, 3], [8, -5, 6, 7])
    >>> np.allclose(q, [28, -44, -14, 48])
    True

    """
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
                     x1*w0 + y1*z0 - z1*y0 + w1*x0,
                     -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                     x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=np.float64)

def quaternion_real(quaternion):
    """Return real part of quaternion.

    >>> quaternion_real([3, 0, 1, 2])
    3.0

    """
    return float(quaternion[0])

def quaternion_imag(quaternion):
    """Return imaginary part of quaternion.

    >>> quaternion_imag([3, 0, 1, 2])
    array([ 0.,  1.,  2.])

    """
    return np.array(quaternion[1:4], dtype=np.float64, copy=True)

def quaternion_transform(quat, v):
    # Transform a vector from the current coordinate frame to a coordinate
    # frame rotated with the quaternion
    r = 2.0 / np.dot(quat, quat)
    qimag = quaternion_imag(quat)
    qr = quaternion_real(quat)
    tmp1 = (r*qr*qr - 1.0)*np.array(v, dtype=np.float64, copy=True)
    tmp2 = (r*np.dot(qimag, v))*qimag
    tmp3 = (r*qr)*np.cross(qimag, v)
    return tmp1 + tmp2 - tmp3

def quaternion_backTransform(quat, v):
    # Transform a vector from the coordinate frame rotated with the
    # quaternion to the current coordinate frame

    r = 2.0 / np.dot(quat, quat)
    qimag = quaternion_imag(quat)
    qr = quaternion_real(quat)
    tmp1 = (r*qr*qr - 1.0)*np.array(v, dtype=np.float64, copy=True)
    tmp2 = (r*np.dot(qimag, v))*qimag
    tmp3 = (r*qr)*np.cross(qimag, v)
    return tmp1 + tmp2 + tmp3

airspeed_mps = 30
alpha = 0
beta = 0
pos_ned = np.array( [0.0, 0.0, 0.0] )
vel_ned = np.array( [10.0, 0.0, 0.0] )
phi_rad = 0.0
the_rad = 0.0
psi_rad = 0.0
ned2body = eul2quat(phi_rad, the_rad, psi_rad)
bax = 0.0
bay = 0.0
baz = 0.0
p = 0
q = 0
r = 0

dt = 0.01
t = 0.0

throttle = 0.7
aileron = 0.0
elevator = 0.0
rudder = 0.0
last_vel_body = None

data = []
while t < 60:
    state = np.array( [ airspeed_mps**2, throttle,
                        aileron, elevator, rudder,
                        math.cos(phi_rad), math.cos(the_rad),
                        math.sin(phi_rad), math.sin(the_rad),
                        alpha, beta,
                        bax, bay, baz,
                        p, q, r ] )
    
    next = A @ state
    #print("state:", state)
    #print("next:", next)
    #print()

    if next[0] > 0:
        airspeed_mps = math.sqrt(next[0])
    else:
        airspeed_mps = 0
    alpha = next[9]
    beta = next[10]
    #bax = next[11]
    #bay = next[12]
    #baz = next[13]
    #p = next[14]
    q = next[15]
    #r = next[16]
    
    # update attitude
    rot_body = eul2quat(p*dt, q*dt, r*dt)
    ned2body = quaternion_multiply(ned2body, rot_body)
    #phi_rad, the_rad, psi_rad = quat2eul(ned2body)
    lock_phi_rad, the_rad, lock_psi_rad = quat2eul(ned2body)

    # velocity in body frame
    bd = math.sin(alpha) * airspeed_mps
    be = math.sin(beta) * airspeed_mps
    if airspeed_mps**2 - bd**2 - be**2 > 0:
        bn = math.sqrt( airspeed_mps**2 - bd**2 - be**2 )
    else:
        bn = 0

    # accel in body frame
    vel_body = np.array( [bn, be, bd] )
    if last_vel_body is None:
        last_vel_body = vel_body.copy()
    accel_body = vel_body - last_vel_body
    last_vel_body = vel_body.copy()
    bax = accel_body[0]
    #bay = accel_body[1]
    baz = accel_body[2]

    # velocity in ned fram
    vel_ned = quaternion_backTransform( ned2body, np.array([bn, be, bd]) )
                                     
    # update position
    pos_ned += vel_ned * dt

    data.append( [t, airspeed_mps, throttle, aileron, elevator, rudder, phi_rad, the_rad, psi_rad, alpha, beta, bax, bay, baz, p, q, r] )
    data[-1].extend( pos_ned.tolist() )
    data[-1].extend( vel_ned.tolist() )
    
    t += dt

data = np.array(data)
print(data[:,0])
plt.figure()
plt.plot( data[:,0], data[:,1], label="Airspeed (mps)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,7]*r2d, label="Pitch (deg)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,9]*r2d, label="alpha (deg)" )
plt.plot( data[:,0], data[:,10]*r2d, label="beta (deg)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,11]*r2d, label="body ax (mps^2)" )
plt.plot( data[:,0], data[:,12]*r2d, label="body ay (mps^2)" )
plt.plot( data[:,0], data[:,13]*r2d, label="body az (mps^2)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,15]*r2d, label="Pitch rate (deg/sec)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,19], label="Pos 'down' (m)" )
plt.legend()
plt.figure()
plt.plot( data[:,0], data[:,20], label="Vel 'north' (m)" )
plt.plot( data[:,0], data[:,21], label="Vel 'east' (m)" )
plt.plot( data[:,0], data[:,22], label="Vel 'down' (m)" )
plt.legend()
plt.show()
