# Notes on the flight control law implementations here

* I don't know what to call the technique I'm using.

* Using a simple least squares fit of flight data terms, I compute a simple
  function to estimate the things we want to control using relevant terms and
  relevent inceptor (control surfaces).  This leads to a system of "n" equations
  and "n" unknowns (usually 2x2 or 1x1) that is easy to put in matrix form and
  solve (or just solve algebraicly and code up.)

* I am sure I have rediscovered some simple controls technique that is already
  well know.  But alas I am just a poor lowly computer science major.

* I understand that "There is nothing new under the sun."

* But I haven't found anything in the literature that connects up directly with
  what I'm doing here.  Either I don't know the right terms to be searching for,
  or I'm doing something crazy/dumb/weird that everyone else decided was not
  worth pursuing.

* Perhaps all the grad school level advanced control theory folks leap frogged
  simple stuff like this and have gone straight to much more exotic and exciting
  techniques?

* Quote of the day: "Build exciting things with boring technology."

* What I am doing here has layers and some complexity, but at the heart it is
  very simple.  I'm doing straight-forward least squares fits of raw flight data
  and then distilling this down to two equations with two unknowns (or one
  equation with one unknown) and that is super easy to solve for.  This is the
  heart of the technique.
