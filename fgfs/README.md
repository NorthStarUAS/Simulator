# FGFS Interface

The FlightGear flight simulator (open-souce) can be launched in a
viewer only mode.  The rc-sim simulation script can send the sim
results to FlightGear via udp packets and FlightGear will show the
visual animation of the sim.

1. Install the rc-sim.xml file into the FlightGear data Protocols subdirectory.

   This is a simple udp packet definition for the minimum set of
   values needed to send to FlightGear to render the sim animation.
   
2. Run FlightGear with the option:

   --generic=socket,in,100,,6504,udp,rc-sim
   --fdm=null

3. Launch the run_sim.py script.  Currently the run_sim.py script is
   reading the attached usb joystick (not FlightGear, although
   FlightGear may see it too.)
