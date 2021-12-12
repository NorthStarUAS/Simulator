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