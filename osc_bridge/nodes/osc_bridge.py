#!/usr/bin/env python

import roslib; roslib.load_manifest('osc_bridge')
import rospy
import sys

from twisted.internet import reactor

from osc_bridge.oscinterface import OscInterface

if __name__ == "__main__":
    def start():
        try:
            OscInterface("Test", 8000)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup."
            reactor.fireSystemEvent('shutdown')

    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
