"""
oscinterface

Contains classes and helper functions for interfacing between ROS and OSC
"""
import roslib
roslib.load_manifest('osc_bridge')
import rospy

from pytouchosc.bonjour import Bonjour

from twisted.internet import reactor

from txosc import osc
from txosc import dispatch
from txosc import async

import sys
import traceback
import threading

class OscClient(object):
    """
    An object to represent a connected OSC Client
    """
    def __init__(self, servicename, hostname, address, port):
        """
        Construct for OscClient object
        
        @type serviceName: C{string} or C{unicode}
        @param serviceName: Bonjour service name of the client
        @type hostname: C{string} or C{unicode}
        @param hostname: Resolved hostname of the client
        @type address: C{string}
        @param address: Resolved IP address of the client
        @type port: C{int}
        @param port: Port that the OSCClient receives on.
        """
        if type(address) is str:
            self.__address = address
        else:
            raise ValueError("Address must be a string")
        if type(port) is int:
            self.__port = port
        else:
            raise ValueError("Port must be an integer")
        if type(hostname) is str:
            self.__hostname = hostname
        elif type(hostname) is unicode:
            self.__hostname = hostname.encode('ascii')
        else:
            raise ValueError("Name must be a string")

        if type(servicename) is str:
            self.__servicename = servicename
        elif type(servicename) is unicode:
            self.__servicename = servicename.encode('ascii')
        else:
            raise ValueError("Servicename must be a string")

    def _get_send_tuple(self):
        """ The C{tuple} (address, port) of the client"""
        return (self.__address, self.__port)
    send_tuple = property(_get_send_tuple)

    def _get_address(self):
        """ The resolved IP address of the client"""
        return self.__address
    address = property(_get_address)

    def _get_hostname(self):
        """ The resolved hostname of the client"""
        return self.__hostname
    hostname = property(_get_hostname)

    def _get_servicename(self):
        """ The Bonjour service name of the client"""
        return self.__servicename
    servicename = property(_get_servicename)

    def _get_port(self):
        """ The receiving port of the client"""
        return self.__port
    port = property(_get_port)


class RosOscReceiver(dispatch.Receiver):
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
        for msg in messages:
            matched = False
            address_list = osc.getAddressParts(msg.address)
            value_list = msg.getValues()
            for callback in self.getCallbacks(msg.address):
                callback(address_list, value_list, client)
                matched = True
            if not matched:
                self.fallback(address_list, value_list, client)


class OscInterface(object):
    """
    Base OSC ROS Node
    
    This class handles the most basic interaction between ROS and the OSC 
    interface.
    
    @ivar bonjourServer: Bonjour registration and browse server
    @ivar _osc_sender: OSC Protocol send interface
    @ivar _osc_receiver: OSC Protocol receiver interface
    """
    def __init__(self, osc_name, osc_port, regtype='_osc._udp', **kwargs):
        """
        Initialize OSCNode.
        
        @type osc_name: C{str}
        @param osc_name: Name of the instantiated ROS node. Namespace for all 
        publishers and subscribers.
        @type osc_port: C{int}
        @param osc_port: Port that the OSC server will listen on.
        @type regtype: C{str}
        @param regtype: An appropriate registration type.  Currently only 
        '_osc._udp' is supported.
        """
        rospy.init_node("osc_interface", **kwargs)
        rospy.core.add_shutdown_hook(self._shutdown_by_ros)
        reactor.addSystemEventTrigger('after', 'shutdown',
                                      self._shutdown_by_reactor)

        self.ros_name = rospy.get_name()
        self.osc_name = rospy.get_param("~osc_name", osc_name)
        self.osc_port = rospy.get_param("~port", osc_port)
        self.osc_regtype = rospy.get_param("~regtype", regtype)
        self.print_fallback = rospy.get_param("~print_fallback", True)

        if self.print_fallback:
            rospy.loginfo("Logging all unhandled messages to rospy.loginfo")

        # Bonjour Server
        self.bonjour_server = Bonjour(self.osc_name, self.osc_port,
                                      self.osc_regtype,
                                      debug=rospy.logdebug,
                                      info=rospy.logdebug,
                                      error=rospy.logdebug)

        self.bonjour_server.setClientCallback(self.bonjour_client_callback)

        reactor.callInThread(self.bonjour_server.run, daemon=True)

        self._clients = {}
        self._clients_lock = threading.Lock()

        # Twisted OSC receiver
        self._osc_receiver = RosOscReceiver()
        listener = async.DatagramServerProtocol(self._osc_receiver)
        self._osc_receiver_port = reactor.listenUDP(self.osc_port,
                                                    listener)

        # Twisted OSC Sender
        self._osc_sender = async.DatagramClientProtocol()
        self._osc_sender_port = reactor.listenUDP(0, self._osc_sender)

        # Add OSC callbacks
        self._osc_receiver.fallback = self.fallback

    def _get_clients(self):
        """
        Dictionary of clients discovered by Bonjour
        """
        with self._clients_lock:
            return copy.copy(self._clients)
    clients = self.property(_get_clients)

    def bonjour_client_callback(self, client_list):
        """
        Callback when Bonjour client list is updated.
        
        @type client_list: C{dict}
        @param client_list: A dictionary of clients
        """
        if type(client_list) is not dict:
            raise ValueError("Bonjour Client Callback requires dict type")
        else:
            with self._clients_lock:
                new = set()
                for service_name, service_dict in client_list.iteritems():
                    new.add(service_name)
                    try:
                        self._clients[service_dict["ip"]] = OscClient(
                                                    service_name,
                                                    service_dict["hostname"],
                                                    service_dict["ip"],
                                                    service_dict["port"])
                    except KeyError:
                        exc_type, exc_value, exc_traceback = sys.exc_info()
                        traceback.print_tb(exc_traceback, limit=1,
                                           file=sys.stdout)
                        traceback.print_exception(exc_type, exc_value,
                                                  exc_traceback, limit=5,
                                                  file=sys.stdout)
                old = set(self._clients.keys())
                for removed in (old - new):
                    del self._clients[removed]

    def fallback(self, address_list, value_list, client_address):
        """
        Fallback handler for otherwise unhandled messages.
        
        @type addressList: C{list}
        @type valueList: C{list}
        @type clientAddress: C{list}
        """
        if self.print_fallback:
            rospy.loginfo(address_list)
            rospy.loginfo(value_list)
            rospy.loginfo(client_address)

    def _shutdown_by_reactor(self):
        """
        Reactor shutdown callback.  Sends a signal to rospy to shutdown the ROS
        interfaces
        """

        rospy.signal_shutdown("Reactor shutting down.")

    def _shutdown_by_ros(self, *args):
        """
        ROS shutdown callback.  Sends a signal to reactor to shutdown.
        """
        reactor.fireSystemEvent('shutdown')
