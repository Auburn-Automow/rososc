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

def load_handler(package, handler_class):
    try:
        roslib.load_manifest(package)
    except:
        rospy.logerr("Failed to load manifest for package %s" % package)
    try:
        (module, cls) = handler_class.split("/")
        m = __import__(package + "." + module, fromlist=[cls])
        handler = m.__getattribute__(cls)
    except:
        rospy.logerr("Cannot import package: %s" % package + "." + module)
        rospy.logerr("sys.path was " + str(sys.path))
        return None
    return handler

if __name__ == "__main__":
    def start():
        try:
            t = TouchOscInterface()
            default = rospy.get_param("~load_default", False)
            handlers = rospy.get_param("~handlers", None)

            loaded_handlers = {}

            if handlers:
                rospy.loginfo("Adding handlers from configuration")
                for handler in handlers:

                    params = rospy.get_param("~" + handler)
                    HandlerClass = load_handler(params['pkg'],
                                                params['class'])
                    aliases = [handler] + params['tabpage_aliases']
                    rospy.loginfo("Adding %s handler with names %s" % (handler, aliases))
                    loaded_handlers[handler] = HandlerClass(t,
                                                           handler,
                                                           aliases)
                    t.register_handler(loaded_handlers[handler])

            if default:
                names = set()
                default_handlers = {}
                for tabpage in t.tabpage_handlers.itervalues():
                    [names.add(n) for n in tabpage.tabpage_names]
                layout_file = rospy.get_param("layout_file", None)
                if not layout_file and not handlers:
                    rospy.logfatal("""No named handlers and no layout file
                        specified, rososc will not operate""")
                    return
                elif not layout_file:
                    rospy.logerr("""No layout file specified, default tabpage
                        handler will not operate.""")
                    return

                layout = pytouchosc.Layout.createFromExisting(layout_file)
                for tp in layout.getTabpageNames():
                    if tp not in names:
                        names.add(tp)
                        default_handlers[tp] = DefaultTabpageHandler(t, layout.getTabpage(tp))
                        t.register_handler(default_handlers[tp])
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')

    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
