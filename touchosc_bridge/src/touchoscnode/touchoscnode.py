import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from txosc import osc
from txosc import dispatch
from txosc import async

import oscnode
from twisted.internet import reactor

class TouchOscClient(oscnode.OscClient):
    def __init__(self, address, port, name):
        super(TouchOscClient, self).__init__(address, port, name)
        self.tabpages = set()
        self.activeTabpage = None
        self.clientType = None
        if self.name.lower().find("ipad") != -1:
            self.clientType = "ipad"
        elif self.name.lower().find("iphone") != -1:
            self.clientType = "ipod"
        elif self.name.lower().find("ipod") != -1:
            self.clientType = "ipod"  
    

class TouchOSCNode(oscnode.OSCNode):
    def __init__(self, name, port, regtype='_osc._udp'):
        super(TouchOSCNode, self).__init__(name, port, regtype)
        
        # Handle the accelerometer data from the iPad
        self._osc_receiver.addCallback("/accxyz", self.accel_cb)
        self.accel_pub = rospy.Publisher(name + '/accel', Imu)
        # Add an empty message to vibrate compatible clients (iPhones)
        self.vibrate_sub = rospy.Subscriber(name + '/vibrate', Empty,
                                            self.vibrateCallback)
        # Add a diagnostics publisher
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        reactor.callLater(1.0, self.diagnosticsUpdate)
        
        self._osc_receiver.addCallback("/*",self.tabPageSwitchCallback)
        self.tabpageHandlers = {}
        
    def diagnosticsUpdate(self):
        diagnosticsMsg = DiagnosticArray()
        diagnosticsMsg.header.stamp = rospy.Time.now()
        diagnosticsMsg.status = []
        clientStatus = DiagnosticStatus()
        clientStatus.level = clientStatus.OK
        clientStatus.name = " ".join([self.name,"Client Status"])
        clientStatus.hardware_id = self.name
        clientStatus.message = "OK"
        clientStatus.values = []
        with self.clientsLock:
            for client in self.clients.itervalues():
                clientStatus.values.append(KeyValue(key=client.getName() + " Type",
                                                    value=client.clientType))
                clientStatus.values.append(KeyValue(key=client.getName() + " Tabpage",
                                                    value=client.activeTabpage))
            if len(self.clients) == 0:
                clientStatus.message = "No clients detected"
        diagnosticsMsg.status.append(clientStatus)
        for tabpage in self.tabpageHandlers.itervalues():
            diagnosticsMsg.status.append(tabpage.updateDiagnostics())
        self.diagnostics_pub.publish(diagnosticsMsg)
        reactor.callLater(1.0, self.diagnosticsUpdate)
        
    
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
        
    def addTabpageHandler(self, tabpageHandler):
        name = tabpageHandler.getTabpageName()
        rospy.loginfo("Adding Tabpage: %s"%name)
        self.tabpageHandlers[name] = tabpageHandler
        self.tabpageHandlers[name].setSender(self.sendToAll,
                                             self.sendToClient,
                                             self.sendToAllOthers)
        tpOscNode = self.tabpageHandlers[name].getOscNode()
        for alias, node in tabpageHandler.getAliasNodes().iteritems():
            rospy.loginfo("\tAdding Alias: %s"%alias)
            self._osc_receiver.addNode(alias, node)
        
    def getTabpageHandlerByName(self, name):
        return self.tabpageHandlers[name]
        
    def tabPageSwitchCallback(self, addressList, valueList, sendAddress):
        tabpage = addressList[0]
        if tabpage != 'ping':
            if self.tabpageHandlers.has_key(tabpage):
                self.tabpageHandlers[tabpage].tabpageActiveCallback(sendAddress)
            for page, handler in self.tabpageHandlers.iteritems():
                if page != tabpage:
                    handler.tabpageClosedCallback(sendAddress)
            for client, clientObject in self.clients.iteritems():
                if client == sendAddress[0]:
                    clientObject.activeTabpage = tabpage
                    clientObject.tabpages.add(tabpage)

    def vibrateCallback(self, msg):
        self.sendToAll(osc.Message("/vibrate"))
    
    def initializeTabpages(self):
        for handler in self.tabpageHandlers.values():
            handler.initializeTabpage()
                
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
                self.clients = {}
                for clientName, clientAddress in clientList.iteritems():
                    try:
                        self.clients[clientAddress["ip"]] = TouchOscClient(clientAddress["ip"],
                                                                           clientAddress["port"],
                                                                           clientName)
                    except KeyError:
                        pass
