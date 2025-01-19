# some notes

Lift = Cl \* r (density) \* q_bar \* Area

* m = 1 aircraft
* r = 1 atmos
* qbar = V^2 / 2
* Area = 1 wing area
* F = ma

F = a (m = 1 aircraft)

Lift = az (when mass = 1 aircraft)

az = Cl \* q_bar \* some constant values

Cl_flapalpha = f(alpha, flaps)  # mostly linear with alpha, but discrete with flap positions

Cl_beta = f(beta)  # linear
Cl_pHat = f(abs(alpha), roll rate)
Cl_qHat = f(pitch rate)
Cl_rHat = f(yaw rate) # not significant?
Cl_alphaDot  # not signficant
Cl_ail = f(alpha, ail)
Cl_elev = f(elev)

Cl_term1 = abs(alpha) \* roll rate / vel
Cl_term2 = pitch rate / vel
Cl_term3 = alpha dot / vel # not significant

Cl_stab = Cl_flapalpha + Cl_beta
Cl_dyn + Cl_term1 + Cl_term2 + Cl_term3
Cl_ctrl = Cl_ail + Cl_elev

Cl = Cl_stab + Cl_dyn + Cl_ctrl
