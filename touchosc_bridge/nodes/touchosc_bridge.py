#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import os
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
            handlers = rospy.get_param("~handlers", None)
            default = rospy.get_param("~load_default",False)
            
            if handlers:
                rospy.loginfo("Adding handlers from configuration")
                for handler in handlers:
                    try:
                        params = rospy.get_param("~" + handler)
                        h = load_handler(params['pkg'], params['class'])
                        t.addTabpageHandler(h, handler, params['tabpage_aliases'])
                    except:
                        rospy.logerr("Could not find matching parameter '%s' for specified handler"%handler)
            
            if default: 
                names = set()
                for tabpage in t.tabpageHandlers.itervalues():
                    [names.add(n) for n in tabpage.getAllTabpageNames()]
                    
                layoutFile = rospy.get_param("layout_file", None)
                layoutPath = rospy.get_param("layout_path", None)
                layouts = rospy.get_param("layouts", None)
                    
                if not layoutFile and not layoutPath:
                    rospy.logerr("Default tabpage requires input parameter 'layout_file' or 'layout_path'")
                    rospy.logerr("Default publishers and subscribers will not be created")
                if layoutPath and not layouts:
                    rospy.logerr("Parameter 'layout_path' found without parameter 'layouts'")
                    rospy.logerr("Default publishers and subscribers will not be created")
                
                if layoutFile:
                    layout = pytouchosc.Layout.createFromExisting(layoutFile)
                    for tp in layout.getTabpageNames():
                        if tp not in names:
                            names.add(tp)
                            t.addTabpageHandler(DefaultTabpageHandler, tp, layout.getTabpage(tp))
                elif layoutPath:
                    for file in layouts:
                        layout = pytouchosc.Layout.createFromExisting(os.path.join(layoutPath,file))
                        for tp in layout.getTabpageNames():
                            if tp not in names:
                                names.add(tp)
                                t.addTabpageHandler(DefaultTabpageHandler, tp, layout.getTabpage(tp))
                            

            reactor.callLater(1.0, t.initializeTabpages)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
