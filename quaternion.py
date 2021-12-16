# a collection of required quaternion functions (some taken directly
# from transformations.py)

import math
import numpy as np

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

def multiply(quaternion1, quaternion0):
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

def transform(quat, v):
    # Transform a vector from the current coordinate frame to a coordinate
    # frame rotated with the quaternion
    r = 2.0 / np.dot(quat, quat)
    qimag = quaternion_imag(quat)
    qr = quaternion_real(quat)
    tmp1 = (r*qr*qr - 1.0)*np.array(v, dtype=np.float64, copy=True)
    tmp2 = (r*np.dot(qimag, v))*qimag
    tmp3 = (r*qr)*np.cross(qimag, v)
    return tmp1 + tmp2 - tmp3

def backTransform(quat, v):
    # Transform a vector from the coordinate frame rotated with the
    # quaternion to the current coordinate frame

    r = 2.0 / np.dot(quat, quat)
    qimag = quaternion_imag(quat)
    qr = quaternion_real(quat)
    tmp1 = (r*qr*qr - 1.0)*np.array(v, dtype=np.float64, copy=True)
    tmp2 = (r*np.dot(qimag, v))*qimag
    tmp3 = (r*qr)*np.cross(qimag, v)
    return tmp1 + tmp2 + tmp3

