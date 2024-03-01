# A data-driven approach to create an explicit discrete time-invariant state space model

Assumes the relevent data and states have been properly collected.  We have sampled the desired states and inputs at a discrete time interval over some time period (and we have this data in hand.)

## Construct a global multi-input, multi-output state transition matrix

## Definitions

* $\mathbf{x}(k)$ = state vector at time $k$
* $\mathbf{u}(k)$ = input vector at time $k$
* $\mathbf{v}(k)$ = combined $\begin{bmatrix}\mathbf{x}(k)\\\mathbf{u}(k)\end{bmatrix}$

## Construct the Training Data

This is a direct least squares solution, not a black box AI deep learning system, but we can think of our input data as the data we use to train our system.  We could later "test" our system against other flights not included in the training set and evaluate the accuracy of our model.

* $n$ = number of data samples
* data = $\begin{bmatrix}\mathbf{v}(0), \mathbf{v}(1), \ldots, \mathbf{v}(n)\end{bmatrix}$
* $\mathbf{X} = \begin{bmatrix}\mathbf{v}(0), \mathbf{v}(1), \ldots, \mathbf{v}(n-1)\end{bmatrix}$ = data[:n-1]
* $\mathbf{Y} = \begin{bmatrix}\mathbf{v}(1), \mathbf{v}(2), \ldots, \mathbf{v}(n)\end{bmatrix}$ = data[1:]

Intuition:

* $\mathbf{X}$ and $\mathbf{Y}$ are constructed directly from flight test data.
* $\mathbf{X}$ can be thought of as the set of "current" states.
* $\mathbf{Y}$ can be thought of as the set of "next" states.
* $\mathbf{X}$ and $\mathbf{Y}$ are identical except the columns are shifted by one position and are trimmed so their dimensions match.
* When $\mathbf{X}$ and $\mathbf{Y}$ are postioned vertically, each column $\mathbf{v}(k)$ in $\mathbf{X}$ aligns with $\mathbf{v}(k+1)$ in $\mathbf{Y}$ where all columns in both matrices come from the same data set.

So ... let's go!

## The Big Equation!

Consider this equation constructed directly from our data. $\mathbf{X}$ and $\mathbf{Y}$ are known and complete, $\mathbf{M}$ is our unknown variable:

$\mathbf{Y} = \mathbf{M} \cdot \mathbf{X}$

> Important! If true, this equation states that there exists some hypothetical matrix $\mathbf{M}$ derived directly from our data that maps all "current" states to "all" next states.

The logical questions to ask at this point are: Is this simple equation
solvable? Does solving the equation product anything that is usable and make any
sense? Can this actually work?!?

Because of how matrix vs. vector multiplication works, if the full matrix equation is true, then a simpler vector in/out version of the equation is also true:

$\mathbf{v}(k+1) = \mathbf{M} \cdot \mathbf{v}(k)$

$\mathbf{M}$ is thus (or would be) a state transition matrix that predicts the
next state of our system from the current state and current input.

In the context of flight dynamics we know we can produce a non-linear flight dynamics model of an aircraft -- left as an exercise for the reader, hah!  We know we can linearize the model at any number of fixed state locations -- also left as an exercise for the reader, double hah!

So generically we can express our non-linear dynamics system as a function (which may actually be a complex collection of functions and algorithms):

$\mathbf{v}(k+1) = f(\mathbf{v}(k), \mathbf{u}(k))$

A flight simulator would iterate on this non-linear function endlessly, feeding in the current pilot input as it changes, and logging (or drawing the result graphically.)

It seems reasonable to assume we can replace $f()$ with a matrix approximation $\mathbf{M}$

## Solve for $\mathbf{M}$



## Limitations