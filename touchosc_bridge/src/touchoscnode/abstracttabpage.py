import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import copy


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
        self.names = [self.tabpageName]
        self.names.extend([alias for alias in self.alias])
        
        self.osc_node = {}
        self.osc_node[self.tabpageName] = {}
        self.osc_node[self.tabpageName][None] = dispatch.AddressNode(self.tabpageName)
        for alias in tabpageAlias:
            self.osc_node[alias] = {}
            self.osc_node[alias][None] = dispatch.AddressNode(alias)

        self.__oscSendToClient = None
        self.__oscSendToAll = None
        self.__oscSendToAllOthers = None
        
        self.ros_publishers = {}
        self.ros_subscribers = {}
        self.activeClients = {}
        
    def getTabpageName(self):
        return self.tabpageName
    
    def getAllTabpageNames(self):
        """
        Return a list of all names for this tabpage, including aliases.
        """
        return self.names
    
    def getNodeName(self):
        return self.nodeName
    
    def getOscNode(self):
        return self.osc_node[self.tabpageName][None]
    
    def getAliasNodes(self):
        nodes = []
        for alias in self.alias:
            nodes.append((alias, self.osc_node[alias][None]))
        return nodes     
    
    def updateDiagnostics(self):
        tabpageStatus = DiagnosticStatus()
        tabpageStatus.level = tabpageStatus.OK
        tabpageStatus.name = self.tabpageName + " Handler"
        tabpageStatus.hardware_id = self.nodeName
        tabpageStatus.message = "OK"
        tabpageStatus.values = []
        for client, type in self.activeClients.iteritems():
            tabpageStatus.values.append(KeyValue(key=client, value = type))
        return tabpageStatus
    
    def setSender(self, sendToAll, sendToClient, sendToAllOthers):
        """
        Set sender functions for the tabpage
        """
        self.__oscSendToClient = sendToClient
        self.__oscSendToAll = sendToAll
        self.__oscSendToAllOthers = sendToAllOthers 
    
    def sendToClient(self, element, client):
        e = copy.copy(element)
        if client in self.activeClients.iterkeys():
            basename = '/' + self.activeClients[client]
            if type(e) is osc.Bundle:
                for m in element.getMessages():
                    m.address = '/'.join([basename,m.address])
            if type(e) is osc.Message:
                e.address = '/'.join([basename,e.address])
            self.__oscSendToClient(e, client)
            
    def sendToAll(self, element):
        exclude = set()
        for client in self.activeClients.iterkeys():
            exclude.add(client)
            self.sendToClient(element, client)
        for name in self.names:
            e = copy.copy(element)
            basename = '/' + name
            if type(e) is osc.Bundle:
                newBundle = osc.Bundle()
                for m in e.getMessages():
                    newBundle.add(osc.Message('/'.join([basename,m.address]),
                                              *m.getValues()))
                self.__oscSendToAllOthers(newBundle, exclude)
            elif type(e) is osc.Message:
                e.address = '/'.join([basename,e.address])
                self.__oscSendToAllOthers(e, exclude) 
    
    def sendToActive(self, element):
        """
        Send an OSC C{Message} or C{Bundle} to all clients on this tabpage.
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle, or a list of messages to be sent.
        """
        for client in self.activeClients:
            self.sendToClient(element, client)
                
    def sendToAllOtherActive(self, element, client):
        """
        Send an OSC C{Message} or C{Bundle} to all other clients on this tabpage.
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle, or a list of messages to be sent.
        @type client: C{tuple}
        @param client: (host, port) tuple with destination to leave out.
        """
        for dest in self.activeClients:
            if dest != client:
                self.sendToClient(element, dest) 
        
    def initializeTabpage(self):
        """
        Called immedeately after tabpage is loaded.  
        
        May be used to set default values of controls.
        """
        pass
        
    def tabpageActiveCallback(self, client, tabpage):
        """
        Callback when a client switches to this tabpage.
        """
        if client[0] not in self.activeClients:
            self.activeClients[client[0]] = tabpage
        print self.activeClients
    
    def tabpageClosedCallback(self, client, tabpage):
        """
        Callback when a client switches to any tabpage that is 
        not this one.
        """
        if client[0] in self.activeClients:
            del self.activeClients[client[0]]
    
    def addOscCallback(self, name, callback):
        for node in self.osc_node.itervalues():
            node[name] = dispatch.AddressNode(name)
            node[name].addCallback("*", callback)
            node[name].addCallback("/*", callback)
            node[name].addCallback("/*/*", callback)
            node[None].addNode(name, node[name])
    