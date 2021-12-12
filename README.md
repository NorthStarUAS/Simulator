# rc-sim

Flight dynamics model creation and simulation--based on a novel data-driven approach.

## Overview

Imagine taking the set of measurable and relevant states at each time
step and stacking them into a single vector.  Now collect all these
state vectors for every time step (except the last step: 0 to n-1)
into a giant matrix called **X**.  Next do something similar with the
same state set of state vectors.  Assemble all but the first state
(1-n) into a giant matrix called **Y**.  **Y** is **X** shifted over
by one position.

Now form a simple matrix algebra equation **Y = A * X**

If we can find a matrix **A** that best satisfies this equation in a
least squares sense, then we have a matrix that will take any current
state and predict the next state.  Cool, huh?

We can quickly solve for the **A** matrix purely by assqembling flight
log data into the matrices **X** and **Y**.  We don't need to know
anything about the aircraft metrics or first order principles or aero
coefficients, we just need the data.

For fluids people, the setup to this process is equivalent to Dynamic
Mode Decomposition, except here we want to solve for the **A** matrix.
DMD uses additional simplifications because it is only concerned with
finding the eigenvalues and eigenvectors of the **A** matrix.