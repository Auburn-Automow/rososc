import roslib; roslib.load_manifest('osc_bridge')
import rospy

import osc_msgs.msg
import osc_msgs.encoding

from pytouchosc.bonjour import Bonjour

from twisted.internet import reactor
from txosc import osc
from txosc import dispatch
from txosc import async
from threading import Thread

class OscReceiver:
    """
    OscReceiver
    """
    def __init__(self,port):
        self.port = port
        self.receiver = dispatch.Receiver()
        self._server_port = reactor.listenUDP(self.port,
                async.DatagramServerProtocol(self.receiver))
        self.receiver.addCallback("/ping",self.ping_handler)
        self.receiver.fallback = self.fallback_handler

        self.receiver_thread = threading.Thread()

    def ping_handler(self, message, address):
        rospy.loginfo("Got /ping from %s" % address)

    def fallback_handler(self, message, address):
        rospy.logdebug("Got unknown message %s from %s"%(message, address))

    def quit_handler(self, message, address):
        rospy.loginfo("quit_handler")
        rospy.loginfo("  Got %s from %s" % (message, address))
        reactor.stop()

class OscSender:
    """
    OscSender
    """
    pass

class OSCBridge:
    def __init__(self, name, port, regtype = '_osc._udp'):
        self.name = name
        self.port = port
        self.regtype = regtype

        #Bonjour Server
        self.bonjourServer = Bonjour(self.name, self.port, self.regtype,
                debug = rospy.logdebug,
                info = rospy.loginfo,
                error = rospy.logerr)

    def run(self):
        self.bonjourServer.run()
        reactor.run()
        rospy.loginfo("Listening on osc.udp://localhost:%s" % (self.port))

    def stop(self):
        self.bonjourServer.shutdown()
        rospy.loginfo("Stopped listening on osc.udp://localhost:%s" % (self.port))


