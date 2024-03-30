import csv
from datetime import datetime
import os
from pathlib import Path

from Core.props import root_node, getNode

# Notice: we defer calling the setup() function until the first call to
# update().  This happens after the first sim loop iteration so the property
# tree is fully populated with all the names/values we wish to log.  We can then
# enumerate the property tree automatically and not be required to list every
# field name we want to log in advance (and then be subject to breakage when
# some code or tree names change or something new is added.)
#
# The csv writer needs to define the column names when the log file is opened,
# so if a property is introduced later after the first sim loop iteration it
# cannot be logged.  A quick fix would be to ensure those properties are created
# before (or during) the first sim loop and they can be inited with an zero or
# empty value if they don't yet have a meaningful state.

class LogMgr():
    def __init__(self):
        self.nodes = []
        self.logdir = "Logs"
        self.counter = 0

    def setup(self):
        logList = root_node.get_flat_list()
        for prop_name in logList:
            parent_name, child_name = os.path.split(prop_name)
            node = getNode(parent_name)
            val_type = ""
            self.nodes.append( [prop_name, child_name, node, val_type] )

        if not os.path.exists(self.logdir):
            Path(self.logdir).mkdir(parents=True, exist_ok=True)

        dt_string = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logfile_name = os.path.join(self.logdir, "log_" + dt_string + ".csv")

        self.csvfile = open(self.logfile_name, "w", newline="")
        self.writer = csv.DictWriter(self.csvfile, fieldnames=logList)
        self.writer.writeheader()
        self.counter = 0

    def update(self):
        if not len(self.nodes):
            self.setup()

        row = {}
        for i, [prop_name, child_name, node, val_type] in enumerate(self.nodes):
            if val_type == "":
                t = node.getType(child_name)
                if t != "unknown":
                    print("assigning", prop_name, "type:", t)
                    self.nodes[i][3] = t
            if val_type == float:
                val = node.getFloat(child_name)
            elif val_type == bool:
                val = node.getInt(child_name)
            elif val_type == str:
                val = node.getString(child_name)
                # val = 0 # some csv systems don't like text/strings?
            else:
                val = 0
            row[prop_name] = val
        self.writer.writerow(row)

        # flush every 50 iterations (once per second)
        self.counter += 1
        if self.counter % 50 == 0:
            self.csvfile.flush()