# Random Notes and Thoughts

Caution: this could still be a rapidly evolving project.  Much of the
information could be out dated.  The notes here reflect my thinking
and best understanding on the day they were written.  For sure I am
learning as I go.  If you see any comments here that are wildly wrong
or misleading, please let me know so I can fix and improve the
information.  Don't feel bad about calling me out on my mistakes.
It's all good and helps me learn new things and make this system
better.

## Dec 30, 2021: Nature of fitting to noisy / imperfect data

We have a bunch of buckets to fit the data into.  When there is noise
or bias or imperfections or external forces we can't fully measure
and account for (like wind) then there will be errors in the fit.  The
system blindly performs a best fit and the result is that all this
unaccounted for/unknown error is divided up into the different
available bins to minimize errors.

This can lead to incorrect relationships (i.e. if the elevator is
always pushing back against an altitude disturbance, we might find
this negative correlation in our fit ... we see increasing elevator
correlates with altitude loss.)

Generally when fit errors occur or inverse relationships are
'discovered' the system will boost some other parameter to
match/balance/cover the error.

The end result is the system does behave correctly and as expected
across the fit region, but extrapolating outside of the region
(especially when some independent parameters are scaled by qbar
(vel^2) can lead to some wild inaccuracies and nonphysical behavior,
possibly math divergence.  This is an expected downside of the method
used here.  We can possibly limit the damage by forcing the parameters
to stay within some std deviation range of the mean, but then when we
hit the edges of our envelope, other weird/non-phsyical/non-continuous
behavior could be observed.

Again, this is one of the major downsides to simply fitting flight
data (versus a more traditional approach of teasing out all the
individual aero coefficients and using those as inputs to a full
simulator model.)

## Dec 29, 2021: Selecting Parameters

This is a huge open issue with all kinds of implications and
interrelated effects.  This is just my current thinking on the matter.

* We need to select parameters that we can directly measure or
estimate with a reasonable expectation of mostly Gaussian error
characteristics.

* On the output side, I want to drive a simulation capable of
real-time or batch runs.  I need to select parameters that are
sufficient for updating the sim state, specifically I need to update
orientation and velocity (and then the updated velocity can be used to
update position.)

* I want plausible, physical, airplanie behavior from the simulation!

**Example 1:**

I can use a combination of EKF velocity in the NED coordinate frame +
a running wind vector estimate to estimate X, Y, Z velocity in the
aircraft body frame.  I can add the XYZ body velocities to the state
vector and predict their next values.  In the simulation, I can take
the next predicted XYZ velocities, rotate them back into the NED
coordinate frame and update the position.

However, there is nothing in the system that imposes any constraints
or correlations between the X, Y, and Z velocities, so potentially you
could find the aircraft sliding sideways with a massive Y velocity, or
backwards with a negative X velocity, and the prediction system will
happily carry on.

Instead, I have switched to modeling total velocity (airspeed), alpha
(angle of attack), and beta (side-slip).  These are still 3 values.  I
can convert back and forth between this form and body XYZ velocities.
However, using airspeed, alpha, and beta as input parameters
additionally imposes some natural constraints on the physical
relationship between the X, Y, and Z body frame velocities.

**Example 2:**

This is crude, believe me, I understand that.  The four major forces
influencing an aircraft are Lift vs. Gravity and Thrust vs. Drag.

Gravity: All our flight controllers estimate (and log) roll, pitch,
and yaw in flight.  We know the gravity vector is constant in the NED
frame of reference so we can use this information to rotate the
gravity vector into the aircraft body frame.  Now we know the amount
of gravity force in all 3 axes: X, Y, and Z.

Body Frame Accelerations: We are able to measure body accelerations on
our IMU, but unfortunately these are fairly biased, even on a
calibrated system, and they include gravity.)  So instead we can
approximate the body accelerations (without gravity) by:

    body_accels = (body_velocity - last_body_velocity) / dt

Lift: Now we can estimate lift (note that negative body acceleration
is "up" due to the coordinate system used, and measured gravity at
zero ned acceleration is -9.81.)

    lift = -body_accel_Z - body_gravity_Z

Side note: in the classic aerospace sense we need coefficient of lift
(Cl) which is dependent on alpha along with qbar (air density *
velocity^2 * 0.5) and wing area to compute lift.  In our case we can
compute lifting force directly with our mass and air density
normalized to a value of 1 to keep things simple.  This means we are
unable to vary mass, cg, air density, etc. in the simulation and the
aircraft will always fly as it was configured and as the air density
(r) existed on the day the test flight was performed.

Thrust: If this already wasn't pretty crude, now comes the real hack.
We don't have a direct way to measure thrust in our small UAVs and we
don't have a detailed motor/prop model we can extract directly from
the flight data, but we do know the throttle command.  As a very crude
estimate of thrust (normalized to a range of [0, 1] I do this:

    thrust = sqrt(throttle_command)

Some systems log voltagea and amps which can be used to compute watts
which is a direct measure of the power put into the propulsion system.
It still doesn't account for prop model, but it would fit better than
throttle command anbd would account for voltage drop over the course
of the flight.  For now I'm dealing with some flight logs that don't
include this data so I'm not using that additional information.

Drag: Now with an estimate of thrust we can estimate drag.  Note that
drag must balance both gravity and thrust in the aircraft X axis:

    drag = body_accel_X + body_gravity_X - thrust

This is a crude approximation.  It depends on mass and wing area
canceling each other out.  Units are nonstandard.  This is ok because
the coefficients in the state transition matrix are fit to produce the
correct parameter estimates in the next state.  (Future work might
include teasing out mass and geometry values and including those
correctly into the math, but for now leaving these out simplifies the
system tremendously.)