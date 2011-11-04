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
        self.__tabpages = set()
        self.__activeTabpage = None
        self.__clientType = None
        
        if name.lower().find("ipad") != -1:
            self.__clientType = "ipad"
        elif name.lower().find("iphone") != -1:
            self.__clientType = "ipod"
        elif name.lower().find("ipod") != -1:
            self.__clientType = "ipod"
        
        self.name = self.name.replace("-"," ")
            
    @apply
    def tabpages():
        doc="""Set of tabpages that have been seen on the client since start"""
        def fget(self):
            return self.__tabpages
        def fset(self,value):
            self.__tabpages = value
        return property(**locals())
     
    @apply
    def activeTabpage():
        doc="""Current open tabpage on client"""
        def fget(self):
            return self.__activeTabpage
        def fset(self,value):
            if value not in self.__tabpages:
                self.__tabpages.add(value)
            self.__activeTabpage = value
        return property(**locals())
    
    @apply
    def clientType():
        doc="""Type of client (ipod/ipad)"""
        def fget(self):
            return self.__clientType
        def fset(self,value):
            self.__clientType = value
        return property(**locals())
    

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
        self.tabpages = set()
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
                clientStatus.values.append(KeyValue(key=client.getName() + " Current",
                                                    value=client.activeTabpage))
                clientStatus.values.append(KeyValue(key=client.getName() + " Tabpages",
                                                    value="\n".join(client.tabpages)))
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
        self.tabpages.add(name)
        rospy.loginfo("Adding Tabpage: %s"%name)
        self.tabpageHandlers[name] = tabpageHandler
        self.tabpageHandlers[name].setSender(self.sendToAll,
                                             self.sendToClient,
                                             self.sendToAllOthers)
        tpOscNode = self.tabpageHandlers[name].getOscNode()
        self._osc_receiver.addNode(name, tpOscNode)
        for alias, node in tabpageHandler.getAliasNodes():
            rospy.loginfo("\tAdding Alias: %s"%alias)
            self.tabpageHandlers[alias] = tabpageHandler
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
                for clientName, clientAddress in clientList.iteritems():
                    new.add(clientAddress["ip"])
                    if not self.clients.has_key(clientAddress["ip"]):
                        self.clients[clientAddress["ip"]] = TouchOscClient(clientAddress["ip"],
                                                                      clientAddress["port"],
                                                                      clientName)
                old = set(self.clients.keys())
                for removed in (old-new):
                    del self.clients[removed]
