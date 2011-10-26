import roslib; roslib.load_manifest('osc_bridge')
import rospy

import osc_msgs.msg
import osc_msgs.encoding

from pytouchosc.bonjour import Bonjour

from twisted.internet import reactor, threads
from txosc import osc
from txosc import dispatch
from txosc import async

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
    def __init__(self, name, port, regtype = '_osc._udp'):
        Node(name)
        self.name = name
        self.port = port
        self.regtype = regtype
        
        #Bonjour Server
        self.bonjourServer = Bonjour(self.name, self.port, self.regtype,
                debug = rospy.logdebug,
                info = rospy.loginfo,
                error = rospy.logerr)
        reactor.callInThread(self.bonjourServer.run,daemon=True)
        
        #Twisted OSC receiver
        self._osc_receiver = dispatch.Receiver()
        self._osc_receiver_port = reactor.listenUDP(self.port, async.DatagramServerProtocol(self._osc_receiver))
        
        self._osc_sender = async.DatagramClientProtocol()
        self._osc_sender_port = reactor.listenUDP(0, self._osc_sender)
          
        #Add OSC callbacks
        self._osc_receiver.addCallback("/quit", self.quit_handler)
 
        self.first_node = dispatch.AddressNode("1")
        self.first_node.addCallback("/xy2", self.xy)
        self._osc_receiver.addNode("1", self.first_node)
    
        self._osc_receiver.fallback = self.fallback   
    
    def xy(self, message, address):
        print "xy handler"
        print message.address
        print message.getValues()
        
    def xyz(self, message, address):
        print "xyz handler"
        print message.address
        print message.getValues()
        
    def send(self, element, client):
        self._osc_sender.send(element, client)
    
    def sendToAll(self, element):
        clients = self.bonjourServer.getClients()
        for client in clients.itervalues():
            self._osc_sender.send(element, (client['ip'], client['port']))     
        
    def quit_handler(self, message, address):
        rospy.loginfo("Got /quit, shutting down")
        reactor.stop()
        
    def fallback(self, message, address):
        print message.address
        print message.getValues()