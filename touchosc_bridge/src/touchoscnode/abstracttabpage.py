import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class AbstractTabpageHandler(object):
    """
    Base class for all TabpageHandlers.  In order to start creating your own Tabpage and handler,
    inherit from this class.
    
    How L{TouchOSCNode} interacts with children of AbstractTabpageHandler:
      1. When the L{TouchOSCNode} starts, it instantiates TabpageHandlers using the constructor
      2. It then calls setSender to set the three major OSC sending features, which are:
       - SendToAll - send messages/bundles to all OSC clients found on the network.
       - SendToClient - send messages/bundles to a single OSC client.
       - SendToAllOthers - helper function - send messages/bundles to all clients but one.
       - The TouchOSC Node then adds AbstractTabpageHandler.osc_node to the OSC receiver, activating
            all specified OSC callbacks for that Tabpage.
       - Approximately 0.5 seconds after the last Tabpage is added, setControls is called
            (useful for clearing a tabpage, or setting it to some defaults)
    """
    def __init__(self, nodeName, tabpageName, tabpageAlias=[]):
        """
        Initialize a TabpageHandler object.
        
        @type nodeName: str
        @param nodeName: Name of ROS node to be used as a prefix for subscribers and publishers
        @type tabpageName: str
        @param tabpageName:  Name of the tabpage
        @param tabpageAlias: List of aliases for the tabpage
        """
        self.nodeName = nodeName
        self.tabpageName = tabpageName
        self.alias = tabpageAlias
        
        self.osc_node = dispatch.AddressNode(self.tabpageName)
        self.alias_nodes = {}
        
        for alias in tabpageAlias:
            self.alias_nodes[alias] = dispatch.AddressNode(alias)
            
        self.osc_send = None
        self.oscSendToClient = None
        self.oscSendToAll = None
        self.oscSendToAllOthers = None
        
        self.ros_publishers = {}
        self.ros_subscribers = {}
        self.osc_nodes = {}
        self.clients = None
        
    def getTabpageName(self):
        return self.tabpageName
    
    def getNodeName(self):
        return self.nodeName
    
    def getOscNode(self):
        return self.osc_node
    
    def getAliasNodes(self):
        return self.alias_nodes
        
    def updateDiagnostics(self):
        tabpageStatus = DiagnosticStatus()
        tabpageStatus.level = tabpageStatus.OK
        tabpageStatus.name = self.tabpageName
        tabpageStatus.hardware_id = self.nodeName
        tabpageStatus.message = "OK"
        return tabpageStatus
    
    def setSender(self, sendToAll, sendToClient, sendToAllOthers):
        """
        Set sender functions for the tabpage
        """
        self.oscSendToClient = sendToClient
        self.oscSendToAll = sendToAll
        self.oscSendToAllOthers = sendToAllOthers 
    
    def oscSendToActive(self, element):
        """
        Send an OSC C{Message} or C{Bundle} to all clients on this tabpage.
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle, or a list of messages to be sent.
        """
        if self.activeClients:
            for client in self.clients.iterkeys():
                if client in self.activeClients:
                    self.oscSendToClient(element, client)
                
    def oscSendToAllOtherActive(self, element, client):
        """
        Send an OSC C{Message} or C{Bundle} to all other clients on this tabpage.
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle, or a list of messages to be sent.
        @type client: C{tuple}
        @param client: (host, port) tuple with destination to leave out.
        """
        if self.activeClients:
            for dest in self.clients.iterkeys():
                if dest[0] in self.activeClients and dest[0] != client[0]:
                    self.oscSendToClient(element, dest) 
        
    def initializeTabpage(self):
        """
        Called immedeately after tabpage is loaded.  
        
        May be used to set default values of controls.
        """
        pass
        
    def tabpageActiveCallback(self, client):
        """
        Callback when a client switches to this tabpage.
        """
        if client[0] not in self.activeClients:
            self.activeClients.add(client[0])
    
    def tabpageClosedCallback(self, client):
        """
        Callback when a client switches to any tabpage that is 
        not this one.
        """
        if client[0] in self.activeClients:
            self.activeClients.remove(client[0])
    
    def addOscCallback(self, name, callback):
        self.osc_nodes[name] = dispatch.AddressNode(name)
        self.osc_nodes[name].addCallback("*", callback)
        self.osc_nodes[name].addCallback("/*", callback)
        self.osc_nodes[name].addCallback("/*/*", callback)
        self.osc_node.addNode(name, self.osc_nodes[name])
        for alias, aliasNode in self.alias_nodes.iteritems():
            aliasNode.addNode(alias, self.osc_nodes[name])
        
        