#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import time

from twisted.internet import reactor

from touchoscnode import TouchOSCNode
from touchoscnode import DefaultTabpageHandler
from touchoscnode import DiagnosticsTabpageHandler
from touchoscnode import TeleopTabpageHandler

import pytouchosc

if __name__=="__main__":
    def start():
        try:         
            name = "TouchOscBridge"
            t = TouchOSCNode(name, port=8000)
            t.addTabpageHandler(TeleopTabpageHandler(name))
            reactor.callLater(0.5, t.initializeTabpages)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
