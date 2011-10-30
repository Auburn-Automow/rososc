import roslib; roslib.load_manifest('touchosc_bridge')

import rospy

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import touchosc_msgs.msg

from abstracttabpage import AbstractTabpageHandler
from std_msgs.msg import String

class DiagMessage(object):
    COLORS = {0: "green", 1: "yellow", 2: "red", -1: "gray"}
    def __init__(self, name):
        self.name = name
        self.kvDict = {}
        self.stamp = None
        self.status = -1
        self.kvDisplay = []
    
    def update(self, stamp, msg):
        self.stamp = stamp
        self.status = msg.level 
        for value in msg.values:
            self.kvDict[value.key] = value.value
        self.kvDisplay = sorted(self.kvDict.keys())
       
    def getColor(self):
        return self.COLORS[self.status]
    
    def getName(self):
        return self.name
    
    def getStatus(self):
        return self.status
    
    def display(self, offset = 0):
        it = 1
        toDisplay = []
        for key in self.kvDisplay:
            message=osc.Message("/diagnostics/key%i"%it,key)
            toDisplay.append(message)
            message=osc.Message("/diagnostics/value%i"%it, self.kvDict[key])
            toDisplay.append(message)
            it +=1
            if it == 9:
                break
        while it < 9:
            message=osc.Message("/diagnostics/key%i"%it,'')
            toDisplay.append(message)
            message=osc.Message("/diagnostics/value%i"%it,'')
            toDisplay.append(message)
            it+=1
        return toDisplay
            
class DiagArray(object):
    def __init__(self):
        self.diagDict = {}
        self.diagDisplay = []
        self.systemStatus = DiagnosticStatus.OK
        self.detailedDisplay = None
        
    def addMessage(self, msg):
        for message in msg.status:
            # If the message has not been seen, add it.
            if not self.diagDict.has_key(message.name):
                self.diagDict[message.name] = DiagMessage(message.name)
            self.diagDict[message.name].update(msg.header.stamp, message)
            self.systemStatus = max(self.systemStatus,message.level)
        self.diagDisplay = sorted(self.diagDict.keys())
    
    def setDetailedDisplay(self, number):
        if (number) <= len(self.diagDisplay):
            name = self.diagDisplay[number-1]
        else:
            return ''
        if self.diagDict.has_key(name):
            self.detailedDisplay = name
        else:
            self.detailedDisplay = None
            raise KeyError
        return name
    
    def displayDetailed(self):
        return self.diagDict[self.detailedDisplay].display()
    
    def display(self, offset = 0):
        """
        Refresh the list of diagnostics messages on the display
        """
        it = 1
        toDisplay = []
        for key in self.diagDisplay:
            item = self.diagDict[key]
            message=osc.Message("/diagnostics/dled%i/color"%it, 
                                item.getColor())
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dled%i"%it,1.0)
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,
                                item.getName())
            toDisplay.append(message)
            it+=1
        while it < 17:
            message=osc.Message("/diagnostics/dled%i/color"%it, 'gray')
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dled%i"%it,0.0)
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,'')
            toDisplay.append(message)
            it +=1
        if self.detailedDisplay is not None:
            toDisplay.extend(self.diagDict[self.detailedDisplay].display())  
        return toDisplay
    
    def clearDisplay(self):
        it = 1
        toDisplay = []
        while it < 17:
            message=osc.Message("/diagnostics/dled%i/color"%it, 'gray')
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dled%i"%it,0.0)
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,'')
            toDisplay.append(message)
            it +=1
        toDisplay.extend(self.__clearDetailedDisplay())
        for d in toDisplay:
            print str(d)
        return toDisplay
    
    def __clearDetailedDisplay(self):
        it = 1
        toDisplay = []
        while it < 9:
            message=osc.Message("/diagnostics/key%i"%it,' ')
            toDisplay.append(message)
            message=osc.Message("/diagnostics/value%i"%it,' ')
            toDisplay.append(message)
            it+=1
        return toDisplay       
                
class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    def __init__(self, nodeName):
        super(DiagnosticsTabpageHandler, self).__init__(nodeName, "diagnostics")
        self.tabpageName = "diagnostics"
        self.msgDict = dict()
        
        self.diagSub = rospy.Subscriber("/diagnostics", 
                                        DiagnosticArray,
                                        self.diag_cb)
        
        name='darray'
        self.osc_nodes[name] = dispatch.AddressNode(name)
        self.osc_nodes[name].addCallback("*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*/*", self.darray_cb)
        self.osc_node.addNode(name, self.osc_nodes[name])
        
        self.diagnostics = DiagArray()
        
    def setControls(self):
        self.display(self.diagnostics.clearDisplay())

    def display(self, toDisplay):
        for message in toDisplay:
            self.osc_send(message)
        
    def diag_cb(self, msg):
        self.diagnostics.addMessage(msg)
        toDisplay = self.diagnostics.display()
        self.display(toDisplay)
        
    def darray_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        if len(addParts) == 4:
            try:
                self.diagnostics.setDetailedDisplay(int(addParts[3]))
                self.display(self.diagnostics.displayDetailed())
            except KeyError:
                pass
            
    