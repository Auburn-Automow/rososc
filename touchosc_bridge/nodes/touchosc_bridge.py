#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys

from twisted.internet import reactor

from touchoscnode import TouchOSCNode
from touchoscnode import DefaultTabpageHandler
from touchoscnode import DiagnosticsTabpageHandler

from pytouchosc.layout import Layout
from pytouchosc.tabpage import Tabpage

if __name__=="__main__":
    def start():
        try:
            layoutPath = rospy.get_param("/touchosc_layout_path")
            rospy.loginfo(layoutPath)
            try:
                layout = Layout.createFromExisting(layoutPath)
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr("Layout file not found")
                sys.exit(1)
            
            name = "Test"
            t = TouchOSCNode(name, port=8000)
            t.addTabpageHandler(DiagnosticsTabpageHandler(name))
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
