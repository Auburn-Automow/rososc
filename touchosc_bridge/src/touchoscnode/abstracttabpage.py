import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch
from txosc import async

class AbstractTabpageHandler(object):
    def __init__(self, nodeName, tabpageName):
        self.nodeName = nodeName
        self.tabpageName = tabpageName
        
        self.osc_node = dispatch.AddressNode(self.tabpageName)
        self.osc_send = None
        
        self.ros_publishers = {}
        self.ros_subscribers = {}
        
    def getTabpageName(self):
        return self.tabpageName
    
    def getNodeName(self):
        return self.nodeName
    
    def getOscNode(self):
        return self.osc_node
    
    def setSender(self,sender):
        self.osc_send = sender