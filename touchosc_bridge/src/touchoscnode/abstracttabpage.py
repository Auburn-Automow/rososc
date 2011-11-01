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
        self.oscSendToClient = None
        self.oscSendToAll = None
        self.oscSendToAllOthers = None
        
        self.ros_publishers = {}
        self.ros_subscribers = {}
        self.osc_nodes = {}
        self.clients = {}
        self.activeClients = {}
        
    def getTabpageName(self):
        return self.tabpageName
    
    def getNodeName(self):
        return self.nodeName
    
    def getOscNode(self):
        return self.osc_node
    
    def setSender(self, sendToAll, sendToClient, sendToAllOthers):
        self.oscSendToClient = sendToClient
        self.oscSendToAll = sendToAll
        self.oscSendToAllOthers = sendToAllOthers 
        self.osc_send = self.oscSendToAll
        
    def setControls(self):
        """
        Called immedeately after tabpage is loaded.  May be used to set default values of controls.
        """
        pass
    
    def updateClients(self, clients):
        self.clients = clients
    
    def addOscCallback(self, name, callback):
        self.osc_nodes[name] = dispatch.AddressNode(name)
        self.osc_nodes[name].addCallback("*", callback)
        self.osc_nodes[name].addCallback("/*", callback)
        self.osc_nodes[name].addCallback("/*/*", callback)
        self.osc_node.addNode(name, self.osc_nodes[name])