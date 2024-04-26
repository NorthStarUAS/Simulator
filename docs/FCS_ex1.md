# Constructing a Data-Driven (Model-Based) Flight Control System for a Fixed Wing Aircraft

## Introduction and Motivation

Many flight control systems are built on the standard PID components which are
well known and well studied.  PID controllers are "error-based" in that they
compute the error between the reference (target) value and the input (current)
value.  They create 3 terms based on this error, a proportional term, an
integral term (sum of the errors over time) and a derivative term (difference
between current error and previous error.)  Each of the 3 terms has a gain (user
adjustable knob) and the sum of these 3 terms is the output (i.e. position of a
control surface.)

Typically the proportional term in a PID drives the output to a stable (but
wrong!) value.  The integral term accumulates this error and pushes the output
against the accumulated error to help drive it to zero.  The derivate term
pushes against the rate the error is changing.  It is very much like a damping
term.

What if the proportoinal term in the PID (which is stable but wrong!) could be
replaced with a term that computes the correct output location directly?
Instead of fishing around to find the correct place to drive the output to zero,
couldn't we just model the system and directly compute the correct output?

What follows is one "simple" way to do this.

## Requirement: flight test data

We start out by needing to collect a bunch of actual test data.  We are going to
fit a simple model to our system.  In this example we have a data aquisition
system that records every paramter at 50hz.  For our example our DAQ collects
flight control surface [commanded] positions, accelerometers, gyros, aircraft
attitude (roll, pitch, yaw), and airspeed.

## Requirement: Domain knowledge and engineering judgement

Every situation and use case is different, so good judgement and knowledge of your domain is important.  How you setup the problem greatly influences how you solve it and what quality solution you can expect in the end.

## Example 1: Simple proof of concept -- Roll control

* $rho = 1.225$ $(kg/m^3)$ (air density)
* $v =$ airspeed $(mps)$
* $\bar{q} = 0.5 * v^2 * rho$ (dynamic pressure)
* $\delta_{ail}$ = aileron deflection (range normalized from -1 to 1)
* $\delta_{rud}$ = rudder deflection (range normalized from -1 to 1)
* $p$ = roll rate (rad/sec)
* $r$ = yaw rate (rad/sec)

Very approximately the control surface effectiveness scales linearly with dynamic pressure $(\bar{q})$ so we build this into our fit.

Given some flight test data, do a least squares fit of roll command $(\delta_{ail}*\bar{q})$ vs. roll rate $(p)$ and the result is:

$$
p = 0.0001258 * \delta_{ail}*\bar{q} - 0.03119
$$

Let's rearrange this equation to compute an aileron command that gives us a
requested roll rate:

$$
\begin{align*}
0.0001258 * \delta_{ail}*\bar{q} &= p + 0.03119 \\
\delta_{ail}*\bar{q} &= 7949.1*(p + 0.03119) \\
\delta_{ail}*\bar{q} &= 7949.1*p + 247.9 \\
\delta_{ail} &= (7949.1*p + 247.9) / \bar{q}
\end{align*}
$$

Now let's plug in some real world numbers to see what we get:

* We want to command a +5 deg/sec roll rate (0.08726 rad/sec)
* We are currently flying at 50 mps (about 100 kts)

$$
(7949.1*0.08726 + 247.9) / (1.225*0.5*50*50) = 0.61489
$$

This means to achieve a 5 deg/sec roll rate, we need to command the ailerons to
a position of 0.61 on a normalized range of [-1, 1].  If our true aileron
deflection range is +/- 10 degrees, then a 6.1 degree deflection gives us our 5
deg/sec roll rate.

* At 75 mps (approx. 150 kts) the computed deflection is only 0.27 (2.7
  degrees in our example.)  This is because as airspeed increases, dynamic pressure increases (by velocity squared!) and so the aileron can generate the same amount of force with less deflection.
* At 35 mps (approx. 70 kts) the computed deflection is 1.25 (out of range!
  beyond our hard stop.)  This means we would go to full deflection, but not be
  able to fully achieve the commanded roll rate.  Usually this is ok, the
  airplane still flies like an airplane and the pilot (or autopilot!) just has
  to wait a little longer to get to the desired bank angle.

So from raw flight test data we can directly derive a simple function that
estimates the required aileron command to produce a specified roll rate given
the current airspeed!  We can skip PID's and PID tuning entirely!  (Well, let's
not get hasty, reality adds complication, but at a very simplistic level, yes!)

Is this a perfect fit across the entire flight envelope?  Definitely it is not,
but it's a good 1st order approximation and is actually flyable.

## Back to PID's for a moment

As I said before, PID controllers are error-based ... the output (control
surface command) is computed from the error between the reference and input
values.  We provide the reference (goal) value and the current (input) value.
The PID computes the error (ref - input) and everything it does follows from
that.  It finds the output value that produces zero error (i.e. we have achieved
the goal.)  This works, and often works really well, but people have discovered
ways to make them work even better.  If we can give the PID controller hints of
what to do, then the error term will be smaller and it can converge to the
correct value quicker.

* Feed forward terms.  For example, we know that in a bank the aircraft needs
  additional up elevator deflection to maintain a steady pitch angle.  We can
  add a feed forward term based on bank angle that pre-adds in the correct
  amount to compensate for the bank.  The feed forward term can be derived from
  experimentatal data collection, or often just by trial and error ... the engineer simply makes it
  bigger or smaller until the system feels about right.

* Gain scheduling. For example as dynamic pressure increases, the amount of
  control surface deflection required to achieve some goal rotation rate
  decreases.  People will scale the gains of their PID based on things like
  airspeed to get more consistent behavior across a wide flight envelope.

* Gain tuning.  As you can see, there are gains for each of the P, I, and D
  terms.  There may be a gain for each feed forward term.  And each of these can
  be scaled by additional gain scheduling terms.

* Filters.  To make things even more complicated, some of the input signals may
  need to be filtered (the sensors are too noisy) and the type of filter and
  amount of filtering (smoothing vs. phase loss) is another set of parameters to
  adjust.

The end result is that PID controller can have a myriad of knobs to tune that each affect the performance of the system in different ways and tuning can become quite complex!

Now go back to the previous section and look at the formula we derived directly from the flight test data.  Can you see that a model-based approach is doing the equivalent of directly computing the feed forward term, and simultaneously computing the correct scaling for airspeed.

## Hybrid Model-Based Controller

Doing a least squares fit of aileron deflection to roll rate as we did in the
simple example above creates a "model" of our system.  It is a very simple
model, but it captures the most dominant term and scales it correctly.  I call
this approach "model-based" because we are creating a model of system (albeit a
fairly simple one.)

Imagine replacing the P term in a PID with our model-based formula.  Instead of
fishing around for the right output, we directly compute it!  For practical
purposes, we maintain the "I" integral term to account for any model fit error.
Yes, please pause for a minute here and imagine that!

We have options for what to do with the "D" derivative term.  When I use my own
experience and engineering judgement, I observe that the "D" derivative term is
similar to a damping term.  In my work I drop the D term from the PID controller
(gain of zero) and include a separate general purpose rate damper that is always
on.

## Example 2: Coupled Roll/Yaw control

Many (most?) aircraft, especially those with dihedral have significant coupling between the roll and yaw axes.  In other words, aileron deflection not only affects roll, but it can also influence yaw.  And likewise, rudder input not only affects yaw, but also affects roll.  In many aircraft aileron and rudder both have significant influce on both roll and yaw.  Often the roll and yaw controllers are developed and tuned independently which is usually fine.  However, here is a little example of one way to deal with two inputs and two outputs simultaneously.

Let's go back up and grab our simple formula for roll rate $p$ as a function of aileron deflection and $\bar{q}$.

$$
p = 0.0001258 * \delta_{ail}*\bar{q} - 0.03119
$$

Let's refit roll rate $p$ as a function of both aileron and rudder inputs.  And
while we are at it, let's fit yaw rate also as a function of both aileron and rudder inputs.  Here is the result (you can see how the fit coefficients change as we add terms):

$$
p = 0.0001343*\delta_{ail}*\bar{q} + 0.00001223*\delta_{rud}*\bar{q} - 0.0312 \\
r = 0.0000513*\delta_{ail}*\bar{q} + 0.00009179*\delta_{rud}*\bar{q} - 0.0007
$$

At any given time, we will know what values we *want* for $p$ and $r$.  We know the value of $\bar{q}$.  Thus we have 2 equations with 2 unknowns.  We can solve this with some basic algebra, or we could turn this into matrix form!

$$
\begin{bmatrix}p\\r\end{bmatrix} = \begin{bmatrix}0.0001343 & 0.00001223\\0.0000513 & 0.00009179\end{bmatrix} \cdot \begin{bmatrix}\delta_{ail}*\bar{q} \\\delta_{rud}*\bar{q}\end{bmatrix} + \begin{bmatrix}-0.0312 \\-0.0007\end{bmatrix}
$$

We can solve this by inverting the 2x2 matrix.

$$
A = \begin{bmatrix}0.0001343 & 0.00001223\\0.0000513 & 0.00009179\end{bmatrix}
$$

$$
A^{-1} = \begin{bmatrix}7844.94 & -1045.10 \\ -4384.25 & 11478.98\end{bmatrix}
$$

Rearranging terms:

$$
A^{-1}\cdot\begin{bmatrix}p\\r\end{bmatrix} - \begin{bmatrix}-0.0312 \\-0.0007\end{bmatrix} = \begin{bmatrix}\delta_{ail}*\bar{q} \\\delta_{rud}*\bar{q}\end{bmatrix}
$$

Let's write this out as individual equations:

$$
\delta_{ail}*\bar{q} = 7844.94*p - 1045.10*r + 0.0312 \\
\delta_{rud}*\bar{q} = -4384.25*p + 11478.98*r + 0.0007
$$

And finally:

$$
\delta_{ail} = (7844.94*p - 1045.10*r + 0.0312) / \bar{q} \\
\delta_{rud} = (-4384.25*p + 11478.98*r + 0.0007) / \bar{q}
$$

So what have we done here?  We have derived directly from flight test data
formulas to simultantiously set aileron and rudder position to achieve the
specified roll rate $p$ and yaw rate $r$.

## Additional Refinements

These are simple models so not perfect.  There is much more work we can do to make this even better.

* We can add more terms. For example: side force ($accel_y$) and ($1/airspeed$)
  can be useful terms to help our model fit even more accurately.  Caution on
  adding too many terms because correlation is not causation ... we want to add
  the terms that we think contribute to causation!
* Notice that when adding more terms like $accel_y$, we know it's value at any
  time, so we aren't adding unknown variables to our simple system of 2
  equations and 2 unknowns, we are simply adding terms that at each time step
  can be evaluated and collapsed into value.
* Knowing our model fit isn't perfect we can add an integral term after our
  model-based output.  This sucks up any remaining error just like the integral
  term in a PID.
* If we know the range of our model error, we can limit the authority (range) of
  the total integral term to something less than full deflection and allow the [auto] pilot to overcome the max possible integral windup.

## Cautions (!) and Things to Think About (?)

* It is important to make sure the A matrix is invertable (but this is done
  offline when the controller model is build, the model matrix is static and
  doesn't change in flight.)
* In the more complicated example above: what if we don't like the balance of
  aileron and rudder actuation.  In a PID we have knobs to turn, but in a
  model-based approach there is nothing to tune so we must live with the model
  as it is fit to the data.
* It is possible to extend this to include elevator and throttle and have 4
  equations with 4 unknowns.  Do we want to do that?  Maybe?  But more likely
  not.  Even though the actual aircraft may have cross coupling between the
  lateral and longitudinal axes, if we build that into the controller the result
  could be some amount of roll and yaw when all we wanted was pitch.  In theory
  with perfect data and a perfect fit, all this would be accounted for, but the
  reality is our data is not perfect, our fit is not perfect, and we could match
  high frequency correlations in the data that we don't want to convey to our
  flight controller.
* Speaking of frequencies ... all of this is done in the time domain!  But
  controls people love to do everything in the frequency domain.
* Speaking of unwanted high frequency correlations and coupling, I have
  experimented with low pass filtering /all/ the input data to fit to the lower
  frequency trends.  I don't have a solid theoretical basis for doing this, but
  it seems to work well.
