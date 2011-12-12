"""
Contains classes and helper functions for interacting between ROS and TouchOSC.
"""

__package__ = 'touchosc_bridge'
__author__ = 'Michael Carroll <carroll.michael@gmail.com>'

import roslib
roslib.load_manifest('touchosc_bridge')
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from touchosc_msgs.msg import Tabpage

from txosc import osc
from txosc import dispatch
from txosc import async

from osc_bridge.oscinterface import OscInterface, OscClient

from twisted.internet import reactor

import copy

def walk_node(parent, sep='/'):
    """
    Walk a node tree for nodes with callbacks.
    
    @param parent: The parent node to walk through.
    @type parent: C{osc.AddressNode}
    @param sep: Separator for path
    @type sep: C{string}
    """
    consumer = [parent]
    foo = {}
    sep = '/'
    while consumer:
        node = consumer.pop(0)
        if len(node._childNodes) == 0 and len(node._callbacks):
            for cb in node._callbacks:
                cbStr = ".".join([cb.__module__, cb.__name__])
                yield (build_path(node, sep), cbStr)
        else:
            for k, v in node._childNodes.iteritems():
                consumer.append(v)

def build_path(node, sep):
    """
    Reconstruct a path by following the parents of each node.
    
    @param node: The address node to reconstruct
    @type node: C{osc.AddressNode}
    @param sep: Separator for path
    @type sep: C{string}
    """
    if node._parent:
        return build_path(node._parent, sep) + sep + node.getName()
    else:
        return ''

class TouchOscClient(OscClient):
    """
    An object to represent a connected TouchOSC Client
    """
    def __init__(self, servicename, hostname, address, port):
        """
        Constructor for TouchOscClient object
        
        @type servicename: C{string} or C{unicode}
        @param servicename: Bonjour service name of the client
        @type hostname: C{string} or C{unicode}
        @param hostname: Resolved hostname of the client
        @type address: C{string}
        @param address: Resolved IP address of the client
        @type port: C{int}
        @param port: Port that the OSCClient receives on.
        """
        super(TouchOscClient, self).__init__(servicename, hostname, address, port)
        self._tabpages = set()
        self._activeTabpage = None
        self._client_type = None

        if self.servicename.lower().find("[iphone]") != -1:
            self._client_type = "ipod"
        elif self.servicename.lower().find("[ipad]") != -1:
            self._client_type = "ipad"

    def add_tabpage(self, tabpage):
        """
        Add a tabpage to the set of known tabpages on the client
        
        @param tabpage: The tabpage name
        @type tabpage: C{str}
        """
        self._tabpages.add(tabpage.strip('/'))

    @property
    def tabpages(self):
        """
        Set of tabpages that have been seen on the client since connection
        @type: C{set}
        """
        return self._tabpages

    @property
    def active_tabpage(self):
        """
        Current open tabpage on the client
        @type: C{string}
        """
        return copy.copy(self._activeTabpage)

    @active_tabpage.setter
    def active_tabpage(self, tabpage):
        self._activeTabpage = tabpage

    @property
    def client_type(self):
        """
        Type of client ("ipod"/"ipad") for determining screen size.
        """
        return self._client_type


class TouchOscInterface(OscInterface):
    """
    Class containing the OSC sender and receiver as well as ROS Publishers and
    Subscribers.
    """
    def __init__(self, osc_name='ROS OSC', osc_port=8000, **kwargs):
        """
        Initialize TouchOscInterface
        
        @type osc_name: C{str}
        @param osc_name: Name of the Bonjour service to register, may be 
        overridden by ROS parameters in the superclass.
        @type osc_port: C{str}
        @param osc_port: Port that the OSC server will listen on, may be 
        overridden by ROS parameters in the superclass.
        """
        super(TouchOscInterface, self).__init__(osc_name, osc_port, **kwargs)

        # Handle the accelerometer data from the device
        if rospy.get_param("~publish_accel", True):
            self._osc_receiver.addCallback("/accxyz", self.cb_osc_accxyz)
            self.accel_pub = rospy.Publisher(self.ros_name + '/accel', Imu)

        # Add an empty message to vibrate compatible clients (iPhones)
        if rospy.get_param("~vibrate", True):
            self.vibrate_sub = rospy.Subscriber(self.ros_name + '/vibrate',
                                                Empty, self.cb_ros_vibrate)
        # Add a diagnostics publisher
        if rospy.get_param("~publish_diag", True):
            self.diagnostics_pub = rospy.Publisher("/diagnostics",
                                                   DiagnosticArray)
            self._diagnostic_status_callbacks = None
            reactor.callLater(1.0, self.cb_diagnostics_update)

        # Add a tabpage listener    
        if rospy.get_param("~tabpage_sub", True):
            self.tabpage_sub = rospy.Subscriber(self.ros_name + '/tabpage',
                                                Tabpage,
                                                self.cb_ros_switch_tabpage)
        # Add a tabpage publisher
        self.tabpage_pub = rospy.Publisher(self.ros_name + '/tabpage', Tabpage)

        self._osc_receiver.addCallback("/*", self.cb_osc_switch_tabpage)

        self.tabpage_handlers = {}
        self.registered_handlers = set()
        rospy.loginfo("Touchosc interface initialized")

    def cb_diagnostics_update(self):
        """
        Callback periodically called to update the diagnostics status of the 
        TouchOscInterface.
        """
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status = []

        # If the callbacks DiagnosticStatus message hasn't been populated,
        # do that now.
        if not self._diagnostic_status_callbacks:
            callback_status = DiagnosticStatus()
            callback_status.level = callback_status.OK
            callback_status.name = " ".join([self.ros_name,
                                             "Registered Callbacks"])
            callback_status.hardware_id = self.ros_name
            callback_status.message = "OK"
            callback_status.values = []
            diags = [(k, v) for k, v in walk_node(self._osc_receiver)]
            rospy.logdebug("Registered Callbacks:")
            for (k, v) in diags:
                rospy.logdebug('{0:<30}{1:<30}'.format(k, v))
                callback_status.values.append(KeyValue(key=k, value=v))
            self._diagnostic_status_callbacks = callback_status
        msg.status.append(self._diagnostic_status_callbacks)

        # Populate the clients DiagnosticStatus message
        diagnostic_status_clients = DiagnosticStatus()
        diagnostic_status_clients.level = diagnostic_status_clients.OK
        diagnostic_status_clients.name = " ".join([self.ros_name,
                                                    "Client Status"])
        diagnostic_status_clients.hardware_id = self.ros_name
        diagnostic_status_clients.message = "Listening on %d" % self.osc_port
        diagnostic_status_clients.values = []
        #TODO: The clients property accessor is not thread-safe, as far as I can
        #tell, but using a lock tends to make bonjour hang.
        clients = self.clients
        for client in clients.itervalues():
            diagnostic_status_clients.values.append(KeyValue(
                                    key=client.address,
                                    value=client.servicename.split("[")[0]))
            diagnostic_status_clients.values.append(KeyValue(
                                    key=client.address + " Type",
                                    value=client.client_type))
            diagnostic_status_clients.values.append(KeyValue(
                                    key=client.address + " Current",
                                    value=client.active_tabpage))
            diagnostic_status_clients.values.append(KeyValue(
                                    key=client.address + " Tabpages",
                                    value=", ".join(client.tabpages)))
        if len(self.clients) == 0:
            diagnostic_status_clients.message = "No clients detected"
        msg.status.append(diagnostic_status_clients)

        # For each registered tabpage handler, get a DiagnosticStatus message.
        for tabpage in self.tabpage_handlers.itervalues():
            msg.status.append(tabpage.cb_diagnostics_update())

        # Publish
        self.diagnostics_pub.publish(msg)
        reactor.callLater(1.0, self.cb_diagnostics_update)

    def register_handler(self, handler):
        """
        Used to register a tabpage handler with the TouchOSC interface.
        
        @param handler: Initialized tabpage handler that will be connected 
        to the Touchosc interface
        @type handler: Class that implements C{AbstractTabpageHandler}
        """
        if handler.handler_name in self.registered_handlers:
            raise ValueError("""Attempted to register two handlers 
                             with the name %s""" % handler.handler_name)
        else:
            self.registered_handlers.add(handler)
            osc_nodes = handler.osc_nodes
            for tabpage_name, node in osc_nodes.iteritems():
                self._osc_receiver.addNode(tabpage_name, node)
                self.tabpage_handlers[tabpage_name] = handler

    def cb_ros_switch_tabpage(self, msg):
        if msg._connection_header['callerid'] != self.ros_name:
            if not msg.tabpage.startswith('/'):
                msg.tabpage = '/' + msg.tabpage
            if msg.header.frame_id in self.clients:
                self.sendToClient(osc.Message(msg.tabpage), client)
            elif msg.header.frame_id == '':
                self.sendToAll(osc.Message(msg.tabpage))

    def cb_osc_accxyz(self, address_list, value_list, send_address):
        """
        Callback for when accel data is received from a device.
        
        Populates a sensor_msgs/Imu message, including the ip address of the
        sending client in the msg.header.frame_id field.
        
        @param address_list: A list with the OSC address parts in it.
        @type address_list: C{list}
        @param value_list: A list with the OSC value arguments in it.
        @type value_list: C{list}
        @param send_address: A tuple with the (ip, port) of the sender.
        @type send_address: C{tuple}
        """
        msg = Imu()
        msg.linear_acceleration.x = value_list[0] * 9.80665
        msg.linear_acceleration.y = value_list[1] * 9.80665
        msg.linear_acceleration.z = value_list[2] * 9.80665

        msg.header.frame_id = send_address[0]
        msg.header.stamp = rospy.Time.now()
        # Covariance was calculated from about 20 minutes of static data
        # Conditions:
        #    * Back down
        #    * Plugged In
        #    * Vibrate Off
        #    * Cell and Wifi On
        # Results:
        #          x                y                z
        # Mean:    0.2934510093    -0.2174349315    -9.8049353269
        # Stdev:   0.0197007054     0.0205649244     0.0259846818
        # Var:     0.0003881178     0.0004229161     0.0006752037
        var = 0.0008
        msg.linear_acceleration_covariance = [var, 0, 0, 0, var, 0, 0, 0, var]
        msg.angular_velocity_covariance = [0.0] * 9
        msg.angular_velocity_covariance[0] = -1.0
        msg.orientation_covariance = msg.angular_velocity_covariance
        self.accel_pub.publish(msg)

    def cb_osc_switch_tabpage(self, address_list, value_list, send_address):
        """
        Callback for when a client switches tabpages.
        
        @param address_list: A list with the OSC address parts in it.
        @type address_list: C{list}
        @param value_list: A list with the OSC value arguments in it.
        @type value_list: C{list}
        @param send_address: A tuple with the (ip, port) of the sender.
        @type send_address: C{tuple}
        """
        # Since this is a wildcard, ignore /ping and /accxyz messages
        new_tabpage = address_list[0]
        if new_tabpage != 'ping' and new_tabpage != 'accxyz':
            clients = self.clients
            for client, clientObject in clients.iteritems():
                if client == send_address[0]:
                    # Check to see if we have that tabpage on record.
                    if new_tabpage not in clientObject.tabpages:
                        clientObject.add_tabpage(new_tabpage)

                    old_tabpage = clientObject.active_tabpage
                    clientObject.active_tabpage = new_tabpage

                    # Send callbacks
                    try:
                        self.tabpage_handlers[old_tabpage].cb_tabpage_closed(
                                                          send_address[0],
                                                          old_tabpage)
                    except KeyError:
                        pass

                    try:
                        self.tabpage_handlers[new_tabpage].cb_tabpage_active(
                                                          send_address[0],
                                                          new_tabpage)
                    except KeyError:
                        pass

            # Publish a Tabpage message with the new_tabpage
            msg = Tabpage()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = send_address[0]
            msg.tabpage = str(new_tabpage)
            self.tabpage_pub.publish(msg)

    def cb_ros_vibrate(self, msg):
        """
        Callback for the ROS /vibrate subscriber.
        
        When called, sends a /vibrate message to all clients, causing capable
        clients (iPhones) to vibrate
        """
        self.sendToAll(osc.Message("/vibrate"))

    def initialize_tabpages(self):
        """
        Called approximately one second after the reactor starts running,
        may be used to set tabpages to some default state
        """
        for handler in self.registered_handlers:
            handler.initialize_tabpage()

    def bonjour_client_callback(self, client_list):
        """
        Callback when Bonjour client list is updated.
        
        Overrides the parent class's bonjour_client_callback to use the
        TouchOscClient object rather than the OscClient object.
        
        Prunes clients that have disconnected as well.
        
        @type client_list: C{dict}
        @param client_list: A dictionary of clients
        """
        if type(client_list) is not dict:
            raise ValueError("Bonjour Client Callback requires dict type")
        else:
            with self._clients_lock:
                new = set()
                old = set(self._clients.keys())
                for service_name, service_dict in client_list.iteritems():
                    new.add(service_dict["ip"])
                    try:
                        self._clients[service_dict["ip"]] = TouchOscClient(
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

                for added in (new - old):
                    for handler in self.registered_handlers:
                        handler.cb_client_connected(added)
                for removed in (old - new):
                    for handler in self.registered_handlers:
                        handler.cb_client_disconnected(removed)
                    del self._clients[removed]
