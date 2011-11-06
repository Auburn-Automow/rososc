import roslib; roslib.load_manifest('osc_bridge')
import rospy

from pytouchosc.bonjour import Bonjour

from twisted.internet import reactor, threads
from txosc import osc
from txosc import dispatch
from txosc.dispatch import AddressNode
from txosc import async

import threading

class OscClient(object):
    def __init__(self, address, port, name):
        if type(address) is str:
            self.address = address
        else:
            raise ValueError("Address must be a string")
        if type(port) is int:
            self.port = port
        else:
            raise ValueError("Port must be an integer")
        if type(name) is str:
            self.name = name
        elif type(name) is unicode:
            self.name = name.encode('ascii')
        else:
            raise ValueError("Name must be a string")
        
    def getSendTuple(self):
        return (self.address, self.port)
    
    def getName(self):
        return self.name.split(".")[0]


class Node:
    """
    Class to represent a ROS Node.  Used to cross-wire ROS and Twisted shutdown signals.
    """
    def __init__(self, *args, **kwargs):
        rospy.init_node(*args, **kwargs)
        rospy.core.add_shutdown_hook(self._shutdown_by_ros)
        reactor.addSystemEventTrigger('after', 'shutdown', self._shutdown_by_reactor)
    
    def _shutdown_by_reactor(self):
        rospy.signal_shutdown("Reactor shutting down.")
        
    def _shutdown_by_ros(self, *args):
        reactor.fireSystemEvent('shutdown')
    
        
class RosReceiver(dispatch.Receiver):
    """
    A class to override the default behavior of dispatch.Receiver from txosc.
    
    This will call callbacks with the signature:
    C{callback(addressList, valuesList, clientAddress)}
    """
    def dispatch(self, element, client):
        """
        Dispatch an element to all matching callbacks.
        
        Executes every callback matching the message address with element as
        argument.  The order in which the callbacks are called is undefined.
        
        @param element: A Message or Bundle.
        @param client: A (host, port) tuple with the originator's address
        """
        if isinstance(element, osc.Bundle):
            messages = element.getMessages()
        else:
            messages = [element]
        for m in messages:
            matched = False
            addressList = osc.getAddressParts(m.address)
            valuesList = m.getValues()
            clientAddress = client
            for c in self.getCallbacks(m.address):
                c(addressList, valuesList, clientAddress)
                matched = True
            if not matched:
                self.fallback(addressList, valuesList, clientAddress)


class OSCNode(object):
    """
    Base OSC ROS Node
    
    This class handles the most basic interaction between ROS and the OSC interface.
    
    @ivar bonjourServer: Bonjour registration and browse server
    @ivar _osc_sender: OSC Protocol send interface
    @ivar _osc_receiver: OSC Protocol receiver interface
    """
    def __init__(self, oscName, oscPort, regtype='_osc._udp', **kwargs):
        """
        Initialize OSCNode.
        
        @type name: C{str}
        @param name: Name of the instantiated ROS node. Namespace for all publishers and subscribers.
        @type port: C{int}
        @param port: Port that the OSC server will listen on.
        @type regtype: C{str}
        @param regtype: An appropriate registration type.  Currently only '_osc._udp' is supported.
        @type printFallback: C{bool}
        @param printFallback: Enable logging unhandled messages to the console.
        """
        Node("osc_node",**kwargs)
        self.name = rospy.get_param("~osc_name", oscName)
        self.port = rospy.get_param("~port", oscPort)
        self.regtype = rospy.get_param("~regtype", regtype)
        self.rosName = rospy.get_name()
        
        self.printFallback = rospy.get_param("~print_fallback", False)
        
        if self.printFallback:
            rospy.loginfo("Logging all unhandled messages to rospy.loginfo")
        
        # Bonjour Server
        self.bonjourServer = Bonjour(self.name, self.port, self.regtype,
                debug=rospy.logdebug,
                info=rospy.logdebug,
                error=rospy.logdebug)
        self.bonjourServer.setClientCallback(self.bonjourClientCallback)
        reactor.callInThread(self.bonjourServer.run, daemon=True)
        
        self.clients = {}
        self.clientsLock = threading.Lock()
        
        # Twisted OSC receiver
        self._osc_receiver = RosReceiver()
        self._osc_receiver_port = reactor.listenUDP(self.port, async.DatagramServerProtocol(self._osc_receiver))
        
        # Twisted OSC Sender
        self._osc_sender = async.DatagramClientProtocol()
        self._osc_sender_port = reactor.listenUDP(0, self._osc_sender)
          
        # Add OSC callbacks
        self._osc_receiver.fallback = self.fallback   
        
    def sendToClient(self, element, client):
        """
        Send an OSC C{Message} or C{Bundle} to a specified client
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle, or a list of messages to be sent.
        @type client: C{tuple}
        @param client: (host, port) tuple with destination address
        """
        if type(client) is tuple: c = client[0]
        else:c = client
        with self.clientsLock:
            if self.clients.has_key(c):
                self._osc_sender.send(element, self.clients[c].getSendTuple())
    
    def sendToAll(self, element):
        """
        Send an OSC C{Message} or C{Bundle to all known clients.
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle.
        """
        for client in self.clients.itervalues():
            self.sendToClient(element, client.getSendTuple())
    
    def sendToAllOthers(self, element, excludeClient):
        """
        Send an OSC C{Message} or C{Bundle} to all known clients except one
        
        @type element: C{Message} or C{Bundle} or C{list}
        @param element: A single message or bundle.
        @type excludeClient: C{tuple}
        @param excludeClient: (host, port) tuple with destination to leave out.
        """
        if self.clients:
            for client, clientObject in self.clients.iteritems():
                
                if client not in excludeClient:
                    self.sendToClient(element, clientObject.getSendTuple()) 
    
    def bonjourClientCallback(self, clientList):
        """
        Callback when Bonjour client list is updated.
        
        @type client: C{dict}
        @param client: A dictionary of clients {name:{ip,port}}
        """
        if type(clientList) is not dict:
            raise ValueError("Bonjour Client Callback requires dict type")
        else:
            with self.clientsLock:
                new = set()
                for clientName, clientAddress in clientList.iteritems():
                    new.add(clientAddress["ip"])
                    if not self.clients.has_key(clientAddress["ip"]):
                        self.clients[clientAddress["ip"]] = TouchOscClient(clientAddress["ip"],
                                                                      clientAddress["port"],
                                                                      clientName)
                old = set(self.clients.keys())
                for removed in (old-new):
                    del self.clients[removed]

    def fallback(self, addressList, valueList, clientAddress):
        """
        Fallback handler for otherwise unhandled messages.
        
        @type addressList: C{list}
        @type valueList: C{list}
        @type clientAddress: C{list}
        """
        if self.printFallback:
            rospy.loginfo(addressList)
            rospy.loginfo(valueList)
            rospy.loginfo(clientAddress)
