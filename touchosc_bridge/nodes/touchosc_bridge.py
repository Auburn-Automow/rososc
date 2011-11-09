#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import os
import time

from twisted.internet import reactor

from touchosc_bridge.touchoscinterface import TouchOscInterface
from touchosc_bridge.simpletabpage import SimpleTabpageHandler

import pytouchosc


if __name__ == "__main__":
    def start():
        try:
            t = TouchOscInterface()
            th = SimpleTabpageHandler(t, "simple", ["1", "2", "3", "4"])
            t.register_handler(th)

        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')

    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
