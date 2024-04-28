# Notes on the flight control law implementations here

* I don't know what to call the technique I'm using.
* I am sure I have rediscovered something simple that is already well know.
* There is nothing new under the sun.
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