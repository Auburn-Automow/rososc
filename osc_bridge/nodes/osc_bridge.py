#!/usr/bin/env python

import roslib; roslib.load_manifest('osc_bridge')
import rospy
import sys

from twisted.internet import reactor

from oscnode import OSCNode

if __name__=="__main__":
    def start():
        try:
            layoutPath = rospy.get_param("/touchosc_layout_path")
            OSCNode(name="Test", port=8000)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()