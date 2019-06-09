#!/usr/bin/env python 

import sys
import os
from roslaunch import node_args
from roslaunch import rlutil
import roslaunch.xmlloader
from rosgraph.names import script_resolve_name

if "-h" in sys.argv or "--help" in sys.argv:
    print("Usage: " + os.path.basename(__file__) + " package launchfile")
    print("prints the paramters the node-executable gets if it is launched with roslaunch and the given launchfile."
        + " A common use case for this is to get the argument to make the gtcreator debugger work properly")
    sys.exit()

launch_files = rlutil.resolve_launch_arguments(sys.argv[1:])
loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
config = roslaunch.config.load_config_default(launch_files, None, loader=loader, verbose=False, assign_machines=False)
nodes = node_args.get_node_list(config)
if len(nodes) > 1:
    print("ERROR: the launchfile contains more than one launchfile!")
    sys.exit()

node = nodes[0]
node_name = script_resolve_name('roslaunch', node)
node_args = node_args.get_node_args(node_name, launch_files)
if len(node_args) < 2:
    print("ERROR: not enough node args!")
    sys.exit()

namespace = node_args[0].split("=")[1]
clean_args=node_args[2:] + ["__ns:=" + namespace]
print ' '.join(clean_args)
