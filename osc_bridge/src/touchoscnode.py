import roslib; roslib.load_manifest('osc_bridge')
import rospy

import osc_msgs.msg
import osc_msgs.encoding

from txosc import osc
from txosc import dispatch
from txosc import async

from oscnode import OSCNode

class TouchOSCNode(OSCNode):
    def __init__(self, layout = None, name, port, regtype='_osc._udp'):
        super(TouchOSCNode, self).__init__(name, port, regtype)
        
        self._osc_receiver.addCallback("/accxyz", self.osc_accel_cb)
        self._osc_receiver.addCallback("/1", self.osc_tabpage_cb)
        self._osc_receiver.addCallback("/TextDemo", self.osc_TextData_cb)
        
    def addTabpage(self, tabpage):
        self.subscriber = rospy.Subscriber()
        pass
    
    def osc_accel_cb(self):
        pass
    
    def osc_tabpage_cb(self):
        pass
    
    def osc_TextData_cb(self, message, address):
        print message.getValues

class AbstractTabpage(object):
    def __init__(self, name):
        
        
    def addTwistedCallbacks(self):
        