# Random Notes and Thoughts

## Dec 30, 2021: Nature of fitting to noisy / imperfect data

We have a bunch of buckets to fit the data into.  When there is noise
or bias or imperfections or external forces we can't fully lmeasure
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
(vel^2) can lead to some wild inaccurancies and non-physical behavior,
possibly math divergence.  This is an expected downside of the method
used here.  We can possibly limit the damage by forcing the parameters
to stay within some std devation range of the mean, but then when we
hit the edges of our envelope, other weird/non-phsyical/non-continuous
behaviour could be observed.

Again, this is one of the major downsides to simply fitting flight
data (versus a more traditional approach of teasing out all the
individual aero coefficients and using those as inputs to a full
simuilator model.)