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

def walkNode(node, path='', sep='/'):
    newPath = path + sep
    if len(node._childNodes) == 0:
        if len(node._callbacks):
            cb = node._callbacks.pop()
            cbStr = ".".join([cb.__module__,cb.__name__])
            print '{0:<30}{1:<30}'.format(newPath + node.getName(),cbStr)
    for k, v in node._childNodes.iteritems():
        if node.getName():
            newpath = sep.join([path,str(node.getName())])
        else:
            newpath = path
        walkNode(v,newpath)

def walk_node(parent, sep='/'):
    """Walk a node tree for nodes with callbacks."""
    consumer = [parent._osc_receiver]
    foo = {}
    sep = '/'
    while consumer:
        node = consumer.pop(0)
        if len(node._childNodes) == 0 and len(node._callbacks):
            cb = node._callbacks.pop()
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
            t = TouchOSCNode(name, port=8000)
            t.addTabpageHandler(TeleopTabpageHandler(name,"teleop",
                                                     ["ipod/teleop","ipad/teleop"]))
            foo = {k: v for k, v in walk_node(t)}
            print foo, 'done'
            reactor.callLater(0.5, t.initializeTabpages)
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "Caught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')  

    reactor.addSystemEventTrigger('before','startup', start)
    reactor.callInThread(rospy.spin)
    reactor.run()
