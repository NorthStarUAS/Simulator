import numpy as np

# a simple alpha estimator fit from flight test data
def alpha_func(flaps_norm, qbar, az_mps2):
    # qbar = 0.5 * vc_mps**2 * 1.225_kg_m3
    if qbar < 1:
        return 0                                           # no airspeed

    x = np.array([1, az_mps2/qbar])

    A0 = np.array( [[ -2.75733057, -1150.65361849 ]] )  # flaps up
    A50 = np.array( [[ -4.82827705,  -956.7511148  ]] )  # flaps 50%
    A100 = np.array( [[ -8.32787384,  -966.11862015 ]] )  # flaps 100%

    y0 = A0 @ x
    y50 = A50 @ x
    y100 = A100 @ x

    xp = [0, 0.5, 1.0]
    fp = [y0[0], y50[0], y100[0]]

    return np.interp(flaps_norm, xp, fp)

# a inverse alpha estimator fit from flight test data, given a steady state
# alpha and qbar, estimate the corresponding az (load factor)
def inv_alpha_func(flaps_norm, alpha_deg, qbar):
    # qbar = 0.5 * vc_mps**2 * 1.225_kg_m3
    if qbar < 1:
        return 0                                      # no airspeed

    x = np.array([qbar, alpha_deg*qbar])

    A0 = np.array( [[-0.00264206, -0.00061865]] )  # flaps up
    A50 = np.array( [[-0.00483375, -0.00071729]] )  # flaps 50%
    A100 = np.array( [[-0.00873206, -0.00071212]] )  # flaps 100%

    y0 = A0 @ x
    y50 = A50 @ x
    y100 = A100 @ x

    xp = [0, 0.5, 1.0]
    fp = [y0[0], y50[0], y100[0]]

    return np.interp(flaps_norm, xp, fp)

# a simple beta estimator fit from flight test data (todo: fit a beta function without rudder)
def beta_func(qbar, ay, r, rudder_cmd, throttle_cmd):
    beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*ay + 7.1957*r + 0.0008*ay*qbar + 0.9769*throttle_cmd
    return beta_deg

