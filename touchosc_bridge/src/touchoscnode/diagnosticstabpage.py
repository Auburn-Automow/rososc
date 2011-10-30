import roslib; roslib.load_manifest('touchosc_bridge')

import rospy

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import touchosc_msgs.msg

from abstracttabpage import AbstractTabpageHandler
from std_msgs.msg import String

class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    COLORS = {0: "green", 1: "yellow", 2: "red"}
    def __init__(self, nodeName):
        super(DiagnosticsTabpageHandler, self).__init__(nodeName, "diagnostics")
        self.tabpageName = "diagnostics"
        self.msgDict = dict()
        
        self.diagSub = rospy.Subscriber("/diagnostics", 
                                        DiagnosticArray,
                                        self.diag_cb)
        
        self.testSub = rospy.Subscriber("/test",
                                        String,
                                        self.test_cb)
        
        name='darray'
        self.osc_nodes[name] = dispatch.AddressNode(name)
        self.osc_nodes[name].addCallback("*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*/*", self.darray_cb)
        self.osc_node.addNode(name, self.osc_nodes[name])
        
        self.diagDisplayDict = {}
        self.kvDisplayDict = {}
        self.detailedView = None
        
        
    def diag_cb(self, msg):
        for status in msg.status:
            self.msgDict[status.name] = [msg.header.stamp,status]
            
        it = 1
        for name,[stamp,status] in self.msgDict.iteritems():
            self.diagDisplayDict[it] = name
            message=osc.Message("/diagnostics/dled%i/color"%it, self.COLORS[status.level])
            self.osc_send(message)
            message=osc.Message("/diagnostics/dled%i"%it,1.0)
            self.osc_send(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,status.name)
            self.osc_send(message)
            it+=1
            if status.name == self.detailedView:
                self.display_kv(status.name)
        while it < 17:
            message=osc.Message("/diagnostics/dled%i/color"%it, 'gray')
            self.osc_send(message)
            message=osc.Message("/diagnostics/dled%i"%it,0.0)
            self.osc_send(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,'')
            self.osc_send(message)
            it +=1  
        
            
    def darray_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        if len(addParts) == 4:
            try:
                self.detailedView = self.diagDisplayDict[int(addParts[3])]
                print self.detailedView
                self.display_kv(self.detailedView)  
            except KeyError:
                pass
            
    def display_kv(self, name):
        (stamp, msg) = self.msgDict[name]
        it = 1
        for keyValue in msg.values:
            message=osc.Message("/diagnostics/key%i"%it,keyValue.key)
            self.osc_send(message)
            message=osc.Message("/diagnostics/value%i"%it, keyValue.value)
            self.osc_send(message)
            it +=1
            if it == 9:
                break
        while it < 9:
            message=osc.Message("/diagnostics/key%i"%it,'')
            self.osc_send(message)
            message=osc.Message("/diagnostics/value%i"%it,'')
            self.osc_send(message)
            it+=1
            
            
    def test_cb(self, msg):
        print self.msgDict
    