#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy
import sys
import time

from twisted.internet import reactor

from touchoscnode import TouchOSCNode

import pytouchosc

def load_pkg_module(package, directory):
    in_path = False
    path = sys.path
    pkg_src = package + '/src'
    for entry in sys.path:
        if pkg_src in entry:
            in_path = True
    if not in_path:
        roslib.load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except:
        rospy.logerr( "Cannot import package: %s"% package )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return m


def walk_node(parent, sep='/'):
    """Walk a node tree for nodes with callbacks."""
    consumer = [parent._osc_receiver]
    foo = {}
    sep = '/'
    while consumer:
        node = consumer.pop(0)
        if len(node._childNodes) == 0 and len(node._callbacks):
            for cb in node._callbacks:
                cbStr = ".".join([cb.__module__,cb.__name__])
                yield (build_path(node, sep), cbStr)
        else:
            for k, v in node._childNodes.iteritems():
                consumer.append(v)

def build_path(node, sep):
    """Reconstruct a path by following the parents of each node."""
    if node._parent:
        return build_path(node._parent, sep) + sep + node.getName()
    else:
        return ''

if __name__=="__main__":
    def start():
        try:         
            name = "TouchOscBridge"
            port = 8000
            t = TouchOSCNode(name, port)
            foo = [(k,v) for k, v in walk_node(t)]
            foo.sort()
            for k,v in foo:
               print '{0:<30}{1:<30}'.format(k,v)        
            reactor.callLater(0.5, t.initializeTabpages)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
