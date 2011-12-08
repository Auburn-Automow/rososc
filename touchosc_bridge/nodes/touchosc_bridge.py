#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import os
import time

from twisted.internet import reactor

from touchosc_bridge.touchoscinterface import TouchOscInterface
from touchosc_bridge.simpletabpage import SimpleTabpageHandler
from touchosc_bridge.defaulttabpage import DefaultTabpageHandler

import pytouchosc

if __name__ == "__main__":
    def start():
        try:
            t = TouchOscInterface()
            default = rospy.get_param("~load_default", False)
            handlers = rospy.get_param("~handlers", None)
            
            if default:
                names = set()
                default_handlers = {}
                for tabpage in t.tabpage_handlers.itervalues():
                    [names.add(n) for n in tabpage.tabpage_names]
                layout_file = rospy.get_param("layout_file", None)
                if not layout_file and not handlers:
                    rospy.logfatal("""No named handlers and no layout file
                        specified, rososc will not operate""")
                elif not layout_file:
                    rospy.logerr("""No layout file specified, default tabpage
                        handler will not operate.""")
                    
                layout = pytouchosc.Layout.createFromExisting(layout_file)
                for tp in layout.getTabpageNames():
                    if tp not in names:
                        names.add(tp)
                        default_handlers[tp] = DefaultTabpageHandler(t,layout.getTabpage(tp))
                        t.register_handler(default_handlers[tp])
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')

    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
