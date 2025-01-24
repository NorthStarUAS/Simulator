# a simple alpha estimator fit from flight test data
def alpha_func(qbar, az, p, q, ax):
    p = 0 # roll rate shows up in our alpha measurement because the alpha vane is at the end of the wing, but let's zero it and ignore it.
    alpha_deg = -6.3792 + 14993.7058/qbar -0.3121*az - 4.3545*p + 5.3980*q + 0.2199*ax
    return alpha_deg

# a simple beta estimator fit from flight test data (todo: fit a beta function without rudder)
def beta_func(qbar, ay, r, rudder_cmd, throttle_cmd):
    beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*ay + 7.1957*r + 0.0008*ay*qbar + 0.9769*throttle_cmd
    return beta_deg

