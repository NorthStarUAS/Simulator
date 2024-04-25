# Data-driven system identification and flight dynamics modeling

## What is System Identification?

System Identification involves measuring the inputs and outputs of a dynamic
system and then using that data to derive coefficients to equations of motion
that compute forces and moments acting on that system.  In this way a simulation
of the system can be constructed that mirrors the true system behavior
mathematically when provided an identical set of inputs to the real system.

## Why do System Identification?

* Create a high fidelity flight simulation model (i.e. for training)
* As a foundation for building and testing flight control laws and strategies.

Accurate dynamics models and well tested flight control laws are important for
everything from DIY drones to commercial jets to military aircraft.  These
concepts are important and helpful for those that are just starting out in
aviation design, to those in education, and to those in well established
aviation industries.

## Parameter Identification

People much smarter than myself have worked out a fairly standard set of
equations of motion governing the dynamics of flight.  These equations are built
around parameters (or tables of parameters) that turn the more generalized
equations into something that models a specific vehicle.

The process of system identification uses wind tunnel tests or real world flight
tests to gather data and estimate these parameters.  The process is rather
involved and usually requires a fair bit of domain knowledge and experience with
some high level math and engineering tools (like optimizers.)

With enough care and effort the end result can be a high quality dynamics model
that captures the essential behaviors and cross couplings in an aircraft, and
furthermore allows changing fundamental properties of the aircraft and
simulation like weight, air density (altitude), center of gravity, even some
geometry variations while maintainting the validity of the simulation.

## Proposed "Simple" System Identification strategy: "Aero DMD v2.0"

This project proposes (and demonstrates) a new simplified method for system
identification.

* I have hunted around and so far not found anyone else doing or publishing or
  proposing a method similar to this, so /please/ if you are reading this and
  know of prior work or similar work, let me know so I can learn too!

* This method is yet not officially named, but as a place holder I am calling it
  Aero DMD v2.0 because (a) it is rooted in some theory and strategy I learned
  while working on a Dynamic Mode Decmoposition project at the University of
  Minnesota (DMD comes out of the fluids area), (b) it's Aero related, and (c)
  I'm on my second iteration of the system so it is better than v1.0.

## Theoretical Background

Let $\mathbf{q}$ be a vector that contains all the relevant states in our
system.  These states need to be things we can measure (like rotational rates
and control surface positions.)  A flight simulator simply implements a function
(or set of functions) $\bf{f}()$ that computes $\mathbf{q_{n+1}}$ (the next
state) from $\mathbf{q_n}$ (the current state.) Generically this can be written
as:

$$
\mathbf{ q_{n+1} = \bf{f}(q_n) }
$$

Flight testing (or testing in a wind tunnel) is a process of measuring some of
the parameters directly and collecting current and next state data to fill in
the specifics of the set of equations represented by $\bf{f}()$

In this project I propose that the equation $\mathbf{ q_{n+1} = \bf{f}(q_n) }$
can be modified and rewritten as:

$$
\mathbf{ q_{n+1} = \bf{A} * q_n }
$$

Where $\bf{A}$ is a matrix derived directly from real world flight test data and
$\mathbf{q}$ is a vector of measurable states.

This project demonstrates how the matrix $\bf{A}$ can be computed directly from
flight test data, and how this matrix can be used to implement a high fidelity
non-linear simulation of the measured vehicle.

## States

It is important to select a set of states that can be measured or estimated from
data that is measured.  It is also important to include the set of states that
are useful for driving a flight simulation.  State selection is critical for
this process to work well and this involves at least some domain specific
knowledge.

How does this shake out?  What states are important to measure and what states
are important for driving a flight simulator?

All states (dependent and independent) must be measured (or estimated) during
real world flight tests.  All the states are used to derive the $\bf{A}$ matrix.

The differentiation between dependent and independent is only important for
flight simulation.  Independent states are those that can be measured directly
(control inputs), or computed directly by the simulation, or estimated using the
same methods used to estimate the paramters from the live flight data.

Notice that these states often include additional information (like qbar) in
order to mimic the math more traditionally used to build up the equations of
motion.  However, more specific conditions like air density, aircraft mass, CG,
and surface areas will be cooked into the $\bf{A}$ matrix coefficients.

### Independent states

* Aileron, Elevator, Rudder commands -- these are multiplied by qbar so their
  effectiveness is scaled with airspeed.
* Lift (estimated from body frame Z acceleration and body frame
  gravity vector Z component.)
* Thrust (crudely estimated as sqrt(throttle) command.)
* Drag (estimated as total forward acceleration in the body frame after gravity
  is removed - thrust.)
* The gravity vector is rotated into the body frame of reference and
  has X (out the nose), Y (out the right wing), and Z (down)
  components.  These states correspond to aircraft pitch and roll.
* Angle of attack (alpha) and Angle of sideslip (beta.)
* New: the rotational rates from the previous time step.

### Dependent states

* Rotational rates (p, q, r)
* Body frame accelerations (with gravity removed.)

That's it, that is pretty much all it is!  By learning how to predict
the next value of these states given the current values we can build a
representative flight dynamics model of the original aircraft
(including many of it's asymmetries and idiosyncracies.)

## Constructing the $\bf{A}$ matrix

Assume you have conducted a real UAV flight and logged the data.  Back
on the ground, you scan the recorded log file and assemble a small set
of measurable (or estimatable) states at each time step into a vector.
Now collect all these individual state vectors for every time step
into a giant matrix called "training data".  Take all the training
data except the last column: (i = 0 to n-1) and call that **X**.  Next
do the same thing, except drop only the first column: (i = 1 to n) and
call this matrix **Y**.  Both **X** and **Y** contain the exact same
state data, just shifted over by one position in time.  Think: **X**
contains all the current states and **Y** contains all the next
states.

Now form a simple matrix algebra equation **Y = A * X**

If we can find a matrix **A** that best satisfies this equation in a
least squares sense, then we have a matrix that will take *all* the
current states and compute an estimate of all the next states.  Cool,
huh?

We know the values of the matrices **X** and **Y**, so we can quickly
solve for the **A** matrix.  We can't directly invert **X** because it
is not square, but there are a number of ways to proceed. I chose to
decompose the **X** matrix using a singular value decomposition (SVD),
then apply a few basic matrix algebra steps to isolate **A** on one
side of the equation ... and that is it -- all done using a couple
lines of numpy code.

For fluids people, the initial construction of this process is
identical to Dynamic Mode Decomposition, except here we want to solve
for the actual **A** matrix.  DMD uses additional simplifications
because it is primarily concerned with finding the leading eigenvalues
and eigenvectors of the **A** matrix.

## Simulation

Once the **A** matrix is computed, this forms the heart of the flight
simulator.

The simulation has a simple update loop:

* The independent variables can be attached to external inputs (like a
  physical joystick) and used to update the current state vector.  The
  simulation keeps a running estimate of aircraft body estimate, so
  gravity can always be rotated into the current body frame.

* The dependent variables start out with some initial estimate (like
  zero) and are updated to the next state estimate produced by
  multiplying the previous state by the **A** matrix.  These estimates
  of the dependent variables get carried forward from step to step.

* With each new state estimation, the aircraft body orientation can be
  updated from the new p, q, r estimates.  Then the updated aircraft
  body frame velocities can be rotated into the NED frame to get NED
  velocities.  And finally the NED position can be updated.  There are
  some additional bookkeeping details, but this is essentially all
  that is needed to create a realistic flight simulation from the best
  fit model (aka the **A** matrix.)

## Quick note on delta t (time interval between states.)

One requirement of this system is that the input state vectors must
have a fixed/constant time update rate (dt).  This allows the **X**
and **Y** matrices (current state and next state matrices) to align
correctly.  This time interval is then cooked into the values of the
state transition matrix.  This dt value is automatically determined
from the input data and saved with the model.  The simulation loads
this fixed dt value with the model and updates the simulation at this
rate when running in real-time mode.

## Limitations / Disclaimers

This system is not a perfect or complete answer to creating flight
simulations from real flight data.  There are many limitations and
disclaimers, here are just a few:

* Only works for in-flight state (no ground reactions modeled.)
* Common variables (like air density, aircraft mass, aircraft cg) get
  cooked directly into the model and cannot be varied in the
  simulation.
* Simulation results are most valid within the range of flight
  conditions that were recorded.  Extrapolating beyond these ranges
  may or may not produce some strange or exaggerated effects.
  Definitely do not depend on useful results outside the range of
  state data collected in the flight.
* The fit (approximation) is linear, so when possible the input
  parameters are scaled for a better linear fit (i.e. multiplying
  certain parameters by qbar.)  But the fit is still linear.
* Traditional aerodynamic coefficients are bypassed. New rotational
  rates and body frame velocities are updated estimated/updated every
  frame.  This creates a model the closely matches the performance of
  the original flight data (good) but it cannot be decomposed into
  coefficient parts as with traditional methods.

## Other use cases

The **A** matrix is built for a specific aircraft at a specific
configuration.  This may not be great for full scale aircraft, but it
is common to fly small battery powered UAV's with a fixed
configuration and payload.

* Integrity monitor: Once the model is created, the system is light
  weight enough to run on a typical small UAV flight controller.  the
  current state can be used to predict the expected next state.  That
  can be compared to the true next state and outliers can be flagged
  when they exceed some error threshold.  In addition the system has
  some simple knowledge of which input states are used to predict the
  dependent states, so it may be possible to further isolate an error
  condition to a specific set of inputs.

* The system will predict X, Y, Z velocities in the body frame of
  reference.  The norm() of this velocity vector is the true airspeed.
  Thus, once the model is created, it can be used to estimate/predict
  a synthetic airspeed without needing an actual airspeed sensors.

* For future work, if wind vector estimates are included in the state
  list, it may be possible to use the system to estimate/predict wind
  in all 3 axes.  This could be an alternative to traditional
  in-flight wind speed estimators.  Or this could be used as a way to
  estimate the vertical velocity of the air column for soaring
  applications.  (Note: not yet attempted, just hypothetical for now.)

* This is a fairly generic approach to estimating next state from
  current state.  It could be used for non-flight related predictions.
  Or additional states could be added to the state vector to include
  more effects for a better performance fit.

* Currently this system is only tested for fixed wing models with an
  airspeed sensor.  I hope to think through what would be involved in
  adapting this to fitting multirotor models.

## References

* "Simulation, Parameter Identification and Flight Safety" - Technischen
  Universität München. <https://www.fsd.ed.tum.de/research/modeling/>

* "Aircraft Flight Dynamics" - Wikipedia.
  <https://en.wikipedia.org/wiki/Aircraft_flight_dynamics>
