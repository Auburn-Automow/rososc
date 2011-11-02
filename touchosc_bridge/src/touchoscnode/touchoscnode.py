import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from txosc import osc
from txosc import dispatch
from txosc import async

from oscnode import OSCNode
from twisted.internet import reactor

import numpy as np
from tf import transformations
import geometry_msgs.msg

class TouchOSCNode(OSCNode):
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
        self.clientTabpages = {}
        
    def diagnosticsUpdate(self):
        diagnosticsMsg = DiagnosticArray()
        diagnosticsMsg.header.stamp = rospy.Time.now()
        diagnosticsMsg.status = []
        clientStatus = DiagnosticStatus()
        clientStatus.level = clientStatus.OK
        clientStatus.name = " ".join([self.name,"Client Status"])
        clientStatus.hardware_id = ""
        clientStatus.message = "OK"
        clientStatus.values = []
        for client, clientName in self.clients.iteritems():
            if self.clientTabpages.has_key(client[0]):
                clientStatus.values.append(KeyValue(key=clientName.split(".")[0], 
                                                value=str(self.clientTabpages[client[0]])))
            else:
                clientStatus.values.append(KeyValue(key=clientName.split(".")[0],
                                                    value=str(None)))
        if len(self.clients) == 0:
            clientStatus.message = "No clients detected"
        diagnosticsMsg.status.append(clientStatus)
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
        self._osc_receiver.addNode(name, tpOscNode)
        
    def tabPageSwitchCallback(self, addressList, valueList, sendAddress):
        tabpage = addressList[0]
        if tabpage != 'ping':
            if self.tabpageHandlers.has_key(tabpage):
                self.tabpageHandlers[tabpage].tabpageActiveCallback(sendAddress)
            for page, handler in self.tabpageHandlers.iteritems():
                if page != tabpage:
                    handler.tabpageClosedCallback(sendAddress)
            for client in self.clients.iterkeys():
                if (client[0] == sendAddress[0]):
                    self.clientTabpages[client[0]] = tabpage
        
    def vibrateCallback(self, msg):
        self.sendToAll(osc.Message("/vibrate"))
    
    def initializeTabpages(self):
        for handler in self.tabpageHandlers.values():
            handler.initializeTabpage()
                
    def bonjourClientCallback(self, clients):
        if type(clients) is not dict:
            raise ValueError("Bonjour Client Callback requires dict type")
        else:
            with self.clientsLock:
                self.clients = clients
            rospy.logdebug("New Client Dictionary: %s"%clients)
            for tabpage in self.tabpageHandlers.itervalues():
                tabpage.updateClients(clients)
        
        
        