import roslib; roslib.load_manifest('osc_bridge')
import rospy

from pytouchosc.bonjour import Bonjour

from twisted.internet import reactor, threads
from txosc import osc
from txosc import dispatch
from txosc import async

import threading

class Node:
    def __init__(self, *args, **kwargs):
        rospy.init_node(*args, **kwargs)
        rospy.core.add_shutdown_hook(self._shutdown_by_ros)
        reactor.addSystemEventTrigger('after', 'shutdown', self._shutdown_by_reactor)
    
    def _shutdown_by_reactor(self):
        rospy.signal_shutdown("Reactor shutting down.")
        
    def _shutdown_by_ros(self, *args):
        reactor.fireSystemEvent('shutdown')

class OSCNode(object):
    def __init__(self, name, port, regtype='_osc._udp'):
        Node(name)
        self.name = name
        self.port = port
        self.regtype = regtype
        
        #Bonjour Server
        self.bonjourServer = Bonjour(self.name, self.port, self.regtype,
                debug=rospy.logdebug,
                info=rospy.logdebug,
                error=rospy.logerr)
        self.bonjourServer.setClientCallback(self.bonjourClientCallback)
        reactor.callInThread(self.bonjourServer.run, daemon=True)
        
        self.clients = None
        self.clientsLock = threading.Lock()
        
        #Twisted OSC receiver
        self._osc_receiver = dispatch.Receiver()
        self._osc_receiver_port = reactor.listenUDP(self.port, async.DatagramServerProtocol(self._osc_receiver))
        
        self._osc_sender = async.DatagramClientProtocol()
        self._osc_sender_port = reactor.listenUDP(0, self._osc_sender)
          
        #Add OSC callbacks
        self._osc_receiver.addCallback("/quit", self.quit_handler)
        self._osc_receiver.fallback = self.fallback   
        
    def sendToClient(self, element, client):
        try:
            dest = self.clients[client]
            self._osc_sender.send(element,(dest['ip'],dest['port']))
        except KeyError:
            pass
    
    def sendToAll(self, element):
        clients = self.bonjourServer.getClients()
        for client in clients.itervalues():
            self._osc_sender.send(element, (client['ip'], client['port']))
    
    def sendToAllOthers(self, element, client):
        for destination,address in self.clients.iteritems():
            if destination != client:
                self._osc_sender.send(element, (address['ip'], address['port'])) 
    
    def bonjourClientCallback(self, client):
        if type(client) is not dict:
            raise ValueError("Bonjour Client Callback requires dict type")
        else:
            with self.clientsLock:
                self.clients = client
            rospy.logdebug("New Client Dictionary: %s"%client)
    
    def quit_handler(self, message, address):
        rospy.loginfo("Got /quit, shutting down")
        reactor.stop()
        
    def fallback(self, message, address):
        pass
