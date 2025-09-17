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
fcs_node = PropertyNode("/fcs")
arm_node = PropertyNode("/fcs/arm")
control_node = PropertyNode("/fcs/control")
refs_node = PropertyNode("/fcs/refs")

def dict2props(props_path, dict_tree):
    node = PropertyNode(props_path)
    for key, value in dict_tree.items():
        if type(value) is dict:
            # recurse
            dict2props(props_path + "/" + key, value)
        elif type(value) is int:
            node.setInt(key, value)
        elif type(value) is float:
            node.setDouble(key, value)
        elif type(value) is bool:
            node.setBool(key, value)
        elif type(value) is str:
            node.setString(key, value)
        else:
            print(key, type(value), value)

def props2dict(node):
    result = dict()
    children = node.getChildren(False)
    for child in children:
        # print("  child:", child)
        if node.isParent(child):
            result[child] = props2dict(node.getChild(child))
        elif node.isInt(child):
            result[child] = node.getInt(child)
        elif node.isUInt(child):
            result[child] = node.getUInt(child)
        elif node.isInt64(child):
            result[child] = node.getInt64(child)
        elif node.isBool(child):
            result[child] = node.getBool(child)
        elif node.isDouble(child):
            result[child] = node.getDouble(child)
        elif node.isString(child):
            result[child] = node.getString(child)
        else:
            print("Unknown data type:", child)
            result[child] = node.getDouble(child)
    return result