# a simple alpha estimator fit from flight test data
def alpha_func(flaps_norm, qbar, az_mps2):
    # qbar = 0.5 * vc_mps**2 * 1.225_kg_m3
    if qbar < 1:
        alpha_deg = 0
    elif flaps_norm <= 0.25:
        alpha_deg = -2.7573 - 1150.6536 * az_mps2 / qbar
    elif flaps_norm <= 0.75:
        alpha_deg = -4.8283 - 956.7511 * az_mps2 / qbar
    else:
        alpha_deg = -8.3279 - 966.1186 * az_mps2 / qbar
    return alpha_deg

# a simple alpha estimator fit from flight test data
def inv_alpha_func(flaps_norm, alpha, qbar):
    # qbar = 0.5 * vc_mps**2 * 1.225_kg_m3
    if qbar < 1:
        alpha_deg = 0
    elif flaps_norm <= 0.25:
        alpha_deg = -2.7573 - 1150.6536 * az_mps2 / qbar
    elif flaps_norm <= 0.75:
        alpha_deg = -4.8283 - 956.7511 * az_mps2 / qbar
    else:
        alpha_deg = -8.3279 - 966.1186 * az_mps2 / qbar
    return alpha_deg

# a simple beta estimator fit from flight test data (todo: fit a beta function without rudder)
def beta_func(qbar, ay, r, rudder_cmd, throttle_cmd):
    beta_deg = -0.3552 - 12.1898*rudder_cmd - 3.5411*ay + 7.1957*r + 0.0008*ay*qbar + 0.9769*throttle_cmd
    return beta_deg

