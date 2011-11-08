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

import oscnode
from twisted.internet import reactor

def walk_node(parent, sep='/'):
    """Walk a node tree for nodes with callbacks."""
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
    """Reconstruct a path by following the parents of each node."""
    if node._parent:
        return build_path(node._parent, sep) + sep + node.getName()
    else:
        return ''

class TouchOscClient(oscnode.OscClient):
    def __init__(self, servicename, hostname, address, port):
        super(TouchOscClient, self).__init__(servicename, hostname, address, port)
        self.__tabpages = set()
        self.__activeTabpage = None
        self.__clientType = None

        if self.servicename.lower().find("[iphone]") != -1:
            self.__clientType = "ipod"
        elif self.servicename.lower().find("[ipad]") != -1:
            self.__clientType = "ipad"

    @apply
    def tabpages():
        doc = """Set of tabpages that have been seen on the client since start"""
        def fget(self):
            return self.__tabpages
        def fset(self, value):
            self.__tabpages = value
        return property(**locals())

    @apply
    def active_tabpage():
        doc = """Current open tabpage on client"""
        def fget(self):
            return self.__activeTabpage
        def fset(self, value):
            if value not in self.__tabpages:
                self.__tabpages.add(value)
            self.__activeTabpage = value
        return property(**locals())

    @apply
    def client_type():
        doc = """Type of client (ipod/ipad)"""
        def fget(self):
            return self.__clientType
        def fset(self, value):
            self.__clientType = value
        return property(**locals())


class TouchOSCNode(oscnode.OSCNode):
    def __init__(self, oscName='ROS OSC', oscPort=8000, regtype='_osc._udp', **kwargs):
        super(TouchOSCNode, self).__init__(oscName, oscPort, regtype, **kwargs)

        # Handle the accelerometer data from the device
        if rospy.get_param("~publish_accel", True):
            self._osc_receiver.addCallback("/accxyz", self.accel_cb)
            self.accel_pub = rospy.Publisher(self.rosName + '/accel', Imu)

        # Add an empty message to vibrate compatible clients (iPhones)
        if rospy.get_param("~vibrate", True):
            self.vibrate_sub = rospy.Subscriber(self.rosName + '/vibrate', Empty,
                                                self.vibrateCallback)
        # Add a diagnostics publisher
        if rospy.get_param("~publish_diag", True):
            self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
            self.__callbackDiagnostic = None
            reactor.callLater(1.0, self.diagnosticsUpdate)

        # Add a tabpage listener    
        if rospy.get_param("~tabpage_sub", True):
            self.tabpage_sub = rospy.Subscriber(self.rosName + '/tabpage', Tabpage,
                                                self.ros_switch_tabpage_cb)

        self.tabpage_pub = rospy.Publisher(self.rosName + '/tabpage', Tabpage)

        self._osc_receiver.addCallback("/*", self.tabPageSwitchCallback)
        self.tabpages = set()
        self.tabpageHandlers = {}

    def diagnosticsUpdate(self):
        diagnosticsMsg = DiagnosticArray()
        diagnosticsMsg.header.stamp = rospy.Time.now()
        diagnosticsMsg.status = []
        if not self.__callbackDiagnostic:
            c = DiagnosticStatus()
            c.level = c.OK
            c.name = " ".join([self.name, "Registered Callbacks"])
            c.hardware_id = self.name
            c.message = "OK"
            c.values = []
            diags = [(k, v) for k, v in walk_node(self._osc_receiver)]
            rospy.logdebug("Registered Callbacks:")
            for (k, v) in diags:
                rospy.logdebug('{0:<30}{1:<30}'.format(k, v))
                c.values.append(KeyValue(key=k, value=v))
            self.__callbackDiagnostic = c
        diagnosticsMsg.status.append(self.__callbackDiagnostic)
        clientStatus = DiagnosticStatus()
        clientStatus.level = clientStatus.OK
        clientStatus.name = " ".join([self.name, "Client Status"])
        clientStatus.hardware_id = self.name
        clientStatus.message = "OK, Listening on %d" % self.port
        clientStatus.values = []
        with self.clientsLock:
            for client in self.clients.itervalues():
                clientStatus.values.append(KeyValue(key=client.getName() + " Type",
                                                    value=client.clientType))
                clientStatus.values.append(KeyValue(key=client.getName() + " Current",
                                                    value=client.activeTabpage))
                clientStatus.values.append(KeyValue(key=client.getName() + " Tabpages",
                                                    value=", ".join(client.tabpages)))
            if len(self.clients) == 0:
                clientStatus.message = "No clients detected"
        diagnosticsMsg.status.append(clientStatus)
        for tabpage in self.tabpageHandlers.itervalues():
            diagnosticsMsg.status.append(tabpage.updateDiagnostics())
        self.diagnostics_pub.publish(diagnosticsMsg)
        reactor.callLater(1.0, self.diagnosticsUpdate)

    def ros_switch_tabpage_cb(self, msg):
        if msg._connection_header['callerid'] != self.rosName:
            if not msg.tabpage.startswith('/'):
                msg.tabpage = '/' + msg.tabpage
            if msg.header.frame_id in self.clients:
                self.sendToClient(osc.Message(msg.tabpage), client)
            elif msg.header.frame_id == '':
                self.sendToAll(osc.Message(msg.tabpage))

    def accel_cb(self, addressList, valueList, sendAddress):
        msg = Imu()
        msg.linear_acceleration.x = valueList[0] * 9.80665
        msg.linear_acceleration.y = valueList[1] * 9.80665
        msg.linear_acceleration.z = valueList[2] * 9.80665

        msg.header.frame_id = sendAddress[0]
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

    def addTabpageHandler(self, tabpageHandler, name, *args):
        self.tabpages.add(name)
        rospy.loginfo("Adding Tabpage: %s" % name)
        rospy.loginfo("Tabpage Handler: %s" % tabpageHandler)
        rospy.loginfo("args: %s", args)
        self.tabpageHandlers[name] = tabpageHandler(self.rosName, name, *args)
        self.tabpageHandlers[name].setSender(self.sendToAll,
                                             self.sendToClient,
                                             self.sendToAllOthers)
        self.tabpageHandlers[name].setClients(self.get)
        tpOscNode = self.tabpageHandlers[name].getOscNode()
        self._osc_receiver.addNode(name, tpOscNode)
        for alias, node in self.tabpageHandlers[name].getAliasNodes():
            rospy.loginfo("\tAdding Alias: %s" % alias)
            self.tabpageHandlers[alias] = self.tabpageHandlers[name]
            self._osc_receiver.addNode(alias, node)

    def getTabpageHandlerByName(self, name):
        return self.tabpageHandlers[name]

    def tabPageSwitchCallback(self, addressList, valueList, sendAddress):
        tabpage = addressList[0]
        alias = []
        if tabpage != 'ping' and tabpage != 'accxyz':
            # Send an activate notification to approriate handler
            if self.tabpageHandlers.has_key(tabpage):
                self.tabpageHandlers[tabpage].tabpageActiveCallback(sendAddress,
                                                                    tabpage)
                alias = self.tabpageHandlers[tabpage].getAllTabpageNames()
                msg = Tabpage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = sendAddress[0]
                msg.tabpage = str(tabpage)
                self.tabpage_pub.publish(msg)

            # Send a closed notification to all other handlers
            for page, handler in self.tabpageHandlers.iteritems():
                if page not in alias:
                    handler.tabpageClosedCallback(sendAddress,
                                                  tabpage)

            # Maintain active tabpage and total tabpage information        
            for client, clientObject in self.clients.iteritems():
                if client == sendAddress[0]:
                    clientObject.activeTabpage = tabpage
                    clientObject.tabpages.add(tabpage)

    def vibrateCallback(self, msg):
        self.sendToAll(osc.Message("/vibrate"))

    def initializeTabpages(self):
        for tabpage in self.tabpages:
            self.tabpageHandlers[tabpage].initializeTabpage()

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
                for serviceName, serviceDict in clientList.iteritems():
                    new.add(serviceDict["ip"])
                    if not self.clients.has_key(serviceDict["ip"]):
                        self.clients[serviceDict["ip"]] = TouchOscClient(serviceName,
                                                                      serviceDict["hostname"],
                                                                      serviceDict["ip"],
                                                                      serviceDict["port"])
                old = set(self.clients.keys())
                for removed in (old - new):
                    del self.clients[removed]
