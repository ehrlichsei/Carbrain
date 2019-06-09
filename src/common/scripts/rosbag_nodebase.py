#!/usr/bin/env python

from common_msgs.srv import *
import rospy
import subprocess
import signal


class RosbagRunner:
    def __init__(self, argv):
        self.argv = argv
        self.proc = None

    def activate_module(self, req):
        if not req.moduleActive:
            rospy.loginfo("stopping module")
            if self.proc != None:
                self.proc.wait()
                self.proc = None
            return ActivationServiceResponse()

        rospy.loginfo("starting module")
        self.proc = subprocess.Popen(argv, stdout=subprocess.PIPE)
        return ActivationServiceResponse()

    def cancel(self, a, b):
        if self.proc != None:
            self.proc.terminate()
            self.proc = None
        rospy.signal_shutdown("finished")
        exit(0)

if __name__ == "__main__":
    rospy.init_node('rosbag_nodebase')
    argv = sys.argv
    argv = [a + '_play' if '__name:=' in a else a for a in argv]
    argv[0] = 'rosbag'
    rosbag_runner =  RosbagRunner(argv)
    s = rospy.Service(rospy.get_name() + '/activate_module', ActivationService, rosbag_runner.activate_module)
    signal.signal(signal.SIGINT, rosbag_runner.cancel)
    signal.signal(signal.SIGTERM, rosbag_runner.cancel)
    rospy.loginfo("Waiting for activation call.")
    rospy.spin()
