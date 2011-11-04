#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import time

from twisted.internet import reactor
from touchoscnode import TouchOSCNode
from touchoscnode import DefaultTabpageHandler
import pytouchosc

def load_handler(package, handlerClass):
    roslib.load_manifest(package)
    try:
        (module, cls) = handlerClass.split("/")
        m = __import__( package + "." + module, fromlist=[cls] )
        handler = m.__getattribute__( cls )
    except:
        rospy.logerr( "Cannot import package: %s"% package + "."+ module )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return handler


if __name__=="__main__":
    def start():
        try:        
            t = TouchOSCNode()
            handlers = rospy.get_param("~handlers")
            for handler in [handlers]:
                try:
                    params = rospy.get_param("~" + handler)
                except:
                    raise KeyError("Could not find matching section %s for specified handler"%handler)
                h = load_handler(params['pkg'], params['class'])
                t.addTabpageHandler(h, handler, params['tabpage_aliases'])

            reactor.callLater(1.0, t.initializeTabpages)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
