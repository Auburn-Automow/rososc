#!/usr/bin/env python

import roslib; roslib.load_manifest('osc_bridge')
import rospy
import sys

from twisted.internet import reactor

from oscbridge import OSCBridge
 

if __name__=="__main__":
    rospy.init_node("osc_bridge")
    rospy.loginfo("ROS OSC Python Node")

    def start():
        try:
            OSCBridge("Test Bridge", 12345)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()