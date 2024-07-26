from lib.props import control_engine_node, control_flight_node, inceptor_node

# pass though (full manual) control: emulates the inceptors being directly
# connected to the control surfaces via cables and pushrods

def pass_through():
    control_engine_node.setFloat("throttle", inceptor_node.getFloat("power"))
    control_flight_node.setFloat("aileron", inceptor_node.getFloat("roll"))
    control_flight_node.setFloat("elevator", inceptor_node.getFloat("pitch"))
    control_flight_node.setFloat("elevator_trim", inceptor_node.getFloat("pitch_trim"))
    control_flight_node.setFloat("rudder", inceptor_node.getFloat("yaw"))
    control_flight_node.setBool("flaps_down", inceptor_node.getBool("flaps_down"))
    control_flight_node.setBool("flaps_up", inceptor_node.getBool("flaps_up"))

