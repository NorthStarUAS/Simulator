from PropertyTree import PropertyNode

# For convenience: predefined project specific property nodes.  These nodes can
# be imported and used directly from anywhere.  Subsystems can make their own
# nodes or lookup and store their own references to existing nodes if they
# prefer.
root_node = PropertyNode("/")

accel_node = PropertyNode("/acceleration")
aero_node = PropertyNode("/aero")
att_node = PropertyNode("/attitude")
engine_node = PropertyNode("/propulsion/engine")
environment_node = PropertyNode("/env")
fcs_node = PropertyNode("/fcs")
ic_node = PropertyNode("/initialize")
mass_node = PropertyNode("/mass")
pos_node = PropertyNode("/position")
vel_node = PropertyNode("/velocity")

# Sensors
airdata_node = PropertyNode("/sensors/airdata")
gps_node = PropertyNode("/sensors/gps")
imu_node = PropertyNode("/sensors/imu")
inceptors_node = PropertyNode("/sensors/inceptors")
power_node = PropertyNode("/sensors/power")

# FCS
control_node = PropertyNode("/fcs/control")