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
    COLORS = {0: "green", 1: "yellow", 2: "red", 3: "gray"}
    def __init__(self, name):
        self.name = name
        self.kvDict = {}
        self.stamp = None
        self.status = 3
        self.kvDisplay = []
        self.offset = 0
    
    def update(self, stamp, msg):
        self.stamp = stamp
        self.status = msg.level
        self.msg = msg 
        for value in msg.values:
            self.kvDict[value.key] = value.value
        self.kvDisplay = sorted(self.kvDict.keys())
       
    def getColor(self):
        return self.COLORS[self.status]
    
    def getName(self):
        return self.name
    
    def getStatus(self):
        return self.status
    
    def setOffset(self, offset):
        self.offset = offset
        
    def getOffset(self):
        return self.offset
    
    def getLength(self):
        return len(self.kvDisplay)
    
    def display(self, nofader = True):
        it = 1
        toDisplay = []
        for key in self.kvDisplay[self.offset:]:
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
        if not nofader:
            if self.getLength() <= 8:
                value = 1.00
            else:
                value = self.offset/float(self.getLength()-8)
            toDisplay.append(osc.Message("/diagnostics/kvfader",value))
        toDisplay.append(osc.Message("/diagnostics/deviceled/color",self.COLORS[self.status]))
        toDisplay.append(osc.Message("/diagnostics/deviceled",1.0))
        toDisplay.append(osc.Message("/diagnostics/name",self.name))
        toDisplay.append(osc.Message("/diagnostics/hardware_id",self.msg.hardware_id))
        toDisplay.append(osc.Message("/diagnostics/message",self.msg.message))
        toDisplay.append(osc.Message("/diagnostics/stamp",str(self.stamp.secs)))
        return toDisplay
            
class DiagArray(object):
    COLORS = {0: "green", 1: "yellow", 2: "red", 3: "gray"}
    def __init__(self, subTopic):
        self.diagDict = {}
        self.diagDisplay = []
        self.systemStatus = DiagnosticStatus.OK
        self.systemStatusClear = rospy.Time.now()
        self.detailedDisplay = None
        self.detailedDisplayIndex = None
        self.offset = 0
        self.subTopic = subTopic # Should be "/diagnostics" or "/diagnostics_agg"
        self.expandName = None
        
    def getOffset(self):
        return self.offset
    
    def setOffset(self, offset):
        self.offset = offset
        
    def getLength(self):
        return len(self.diagDisplay)
    
    def setExpandName(self, name):
        self.expandName = name
        
    def getExpandName(self):
        return self.expandName
    
    def getNameByIndex(self, index):
        if index is None:
            return None
        
        if index <= len(self.diagDisplay):
            name = self.diagDisplay[index-1]
            return name
        
        else:
            return None    
    
    def addMessage(self, msg):
        if msg is not None:
            if (rospy.Time.now() - self.systemStatusClear).secs > 60:
                self.systemStatus = 0
                self.systemStatusClear = rospy.Time.now()
            
            for message in msg.status:
                # If the message has not been seen, add it.
                if not self.diagDict.has_key(message.name):
                    self.diagDict[message.name] = DiagMessage(message.name)
                self.diagDict[message.name].update(msg.header.stamp, message)
                if message.level != 3:
                    self.systemStatus = max(self.systemStatus,message.level)   
        if self.subTopic == "/diagnostics_agg":
            self.diagDisplay = []
            if self.expandName:
                self.diagDisplay.append(self.expandName)
            for key in self.diagDict.iterkeys():
                if self.expandName:
                    if key.find(self.expandName) != -1 and key != self.expandName:
                        self.diagDisplay.append(key)
                else:
                    if key.count("/") == 1:
                        self.diagDisplay.append(key)
        else:
            self.diagDisplay = sorted(self.diagDict.keys())
        
    def getSystemStatus(self):
        return self.COLORS[self.systemStatus]
    
    def setDetailedDisplay(self, number):
        if number is None:
            self.detailedDisplay = None
            self.detailedDisplayIndex = None
            return ''
        
        if (number) <= len(self.diagDisplay):
            name = self.diagDisplay[number-1]
        else:
            return ''
        
        if self.diagDict.has_key(name):
            self.detailedDisplay = name
            self.detailedDisplayIndex = number
        else:
            self.detailedDisplay = None
            raise KeyError
        
        return name
    
    def getDetailedDisplayIndex(self):
        return self.detailedDisplayIndex
    
    def getDetailedDisplay(self):
        try:
            val = self.diagDict[self.detailedDisplay]
        except KeyError:
            val = None
        return val
    
    def displayDetailed(self,nofader=True):
        it = 1
        toDisplay = []
        for key in self.diagDisplay[self.offset:]:
            if key == self.getExpandName():
                message=osc.Message("/diagnostics/dlabel%i/color"%it,"orange")
            elif key == self.detailedDisplay:
                message=osc.Message("/diagnostics/dlabel%i/color"%it,"blue")
            else:
                message=osc.Message("/diagnostics/dlabel%i/color"%it,"gray")
            toDisplay.append(message)
            it+=1
        if self.detailedDisplay is not None:
            toDisplay.extend(self.diagDict[self.detailedDisplay].display(nofader))
        else:
            toDisplay.extend(self.__clearDetailedDisplay())
        return toDisplay
    
    def display(self, nofader = True):
        """
        Refresh the list of diagnostics messages on the display
        """
        it = 1
        toDisplay = []
        for key in self.diagDisplay[self.offset:]:
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
        if not nofader:
            if self.getLength() <= 16:
                value = 1.00
            else:
                value = self.offset/float(self.getLength()-16)
            toDisplay.append(osc.Message("/diagnostics/dfader",value))  
        toDisplay.extend(self.displayDetailed(nofader))
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
            message=osc.Message("/diagnostics/dlabel%i/color"%it,"gray")
            toDisplay.append(message)
            it +=1
        toDisplay.append(osc.Message("/diagnostics/dfader",0.0))
        toDisplay.append(osc.Message("/diagnostics/statusled",0.0))
        toDisplay.append(osc.Message("/diagnostics/rostime",""))
        toDisplay.append(osc.Message("/diagnostics/pausetime",""))
        toDisplay.append(osc.Message("/diagnostics/pause",0.0))
        toDisplay.extend(self.__clearDetailedDisplay())
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
        toDisplay.append(osc.Message("/diagnostics/deviceled/color","gray"))
        toDisplay.append(osc.Message("/diagnostics/deviceled",0.0))
        toDisplay.append(osc.Message("/diagnostics/name",""))
        toDisplay.append(osc.Message("/diagnostics/hardware_id",""))
        toDisplay.append(osc.Message("/diagnostics/message",""))
        toDisplay.append(osc.Message("/diagnostics/stamp",""))
        toDisplay.append(osc.Message("/diagnostics/kvfader",0.0))
        return toDisplay       
                
class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    def __init__(self, nodeName):
        super(DiagnosticsTabpageHandler, self).__init__(nodeName, "diagnostics")
        self.msgDict = dict()
        
        self.subTopic = "/diagnostics"
        self.diagSub = rospy.Subscriber(self.subTopic, 
                                        DiagnosticArray,
                                        self.diag_cb)
        
        name='darray'
        self.osc_nodes[name] = dispatch.AddressNode(name)
        self.osc_nodes[name].addCallback("*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*", self.darray_cb)
        self.osc_nodes[name].addCallback("/*/*", self.darray_cb)
        self.osc_node.addNode(name, self.osc_nodes[name])
        
        self.osc_node.addCallback('/kvup',self.kv_updown_cb)
        self.osc_node.addCallback('/kvdown',self.kv_updown_cb)
        self.osc_node.addCallback('/kvfader',self.kv_updown_cb)
        
        self.osc_node.addCallback('/dup',self.d_updown_cb)
        self.osc_node.addCallback('/ddown',self.d_updown_cb)
        self.osc_node.addCallback('/dfader',self.d_updown_cb)
        
        self.osc_node.addCallback('/diagsw',self.change_subscriber)
        
        self.osc_node.addCallback('/pause',self.pause_cb)
        
        self.diagnostics = DiagArray(self.subTopic)
        
        self.paused = False
        
    def setControls(self):
        self.display(self.diagnostics.clearDisplay())
        self.display([osc.Message("/diagnostics/statusled",1.0)])
        self.display([osc.Message("/diagnostics/diaglbl",self.subTopic)])

    def display(self, toDisplay):
        for message in toDisplay:
            self.osc_send(message)
        
    def diag_cb(self, msg):
        if self.paused:
            self.diagnostics.addMessage(None)
            self.display([osc.Message("/diagnostics/pausetime", str((rospy.Time.now()-self.paused).secs))])
        else:
            self.diagnostics.addMessage(msg)
        toDisplay = self.diagnostics.display(nofader=False)
        self.display(toDisplay)
        self.display([osc.Message("/diagnostics/statusled/color",self.diagnostics.getSystemStatus()),
                      osc.Message("/diagnostics/statusled",1.0)])
        self.display([osc.Message("/diagnostics/rostime", str(rospy.Time.now().secs))])
        
    def darray_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        if len(addParts) == 4:
            index = int(addParts[3]) + self.diagnostics.getOffset()
            if self.subTopic == "/diagnostics":
                try:
                    self.diagnostics.setDetailedDisplay(index)
                    detailedDisplay = self.diagnostics.getDetailedDisplay()
                    if detailedDisplay is not None:
                        detailedDisplay.setOffset(0)
                except KeyError:
                    pass
            else:
                if self.diagnostics.getDetailedDisplayIndex() == index:
                    if self.diagnostics.getNameByIndex(index) == self.diagnostics.getExpandName():
                        self.diagnostics.setExpandName(None)
                        self.diagnostics.setDetailedDisplay(None)
                    elif not self.diagnostics.getExpandName():
                        self.diagnostics.setExpandName(self.diagnostics.getNameByIndex(index))
                        self.diagnostics.setDetailedDisplay(None)
                else:
                    self.diagnostics.setDetailedDisplay(index)
                    detailedDisplay = self.diagnostics.getDetailedDisplay()
                    if detailedDisplay is not None:
                        detailedDisplay.setOffset(0)
            toDisplay = self.diagnostics.display(nofader=False)
            self.display(toDisplay)
            
    def kv_updown_cb(self, message, address):
        detailedDisplay = self.diagnostics.getDetailedDisplay()
        if detailedDisplay is None:
            return
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        offset = detailedDisplay.getOffset()
        maxOffset = detailedDisplay.getLength() - 8
        maxOffset = 0 if maxOffset < 0 else maxOffset
        minOffset = 0
        if addParts[1] == 'kvdown':
            if offset < maxOffset:
                detailedDisplay.setOffset(offset + 1)
            nofader = False
        elif addParts[1] == 'kvup':
            if offset > minOffset:
                detailedDisplay.setOffset(offset - 1)
            nofader = False
        elif addParts[1] == 'kvfader':
            detailedDisplay.setOffset(int(round(value[0]*maxOffset)))
            nofader = True
        self.display(detailedDisplay.display(nofader))
        
    def d_updown_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        offset = self.diagnostics.getOffset()
        maxOffset = self.diagnostics.getLength() - 16
        maxOffset = 0 if maxOffset < 0 else maxOffset
        minOffset = 0
        
        if addParts[1] == 'ddown':
            if offset < maxOffset:
                self.diagnostics.setOffset(offset + 1)
            nofader = False
        elif addParts[1] == 'dup':
            if offset > minOffset:
                self.diagnostics.setOffset(offset - 1)
            nofader = False
        elif addParts[1] == 'dfader':
            self.diagnostics.setOffset(int(round(value[0]*maxOffset)))
            nofader = True
        self.display(self.diagnostics.display(nofader))
        
    def change_subscriber(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        if addParts[1] == 'diagsw' and value[0] == 1:
            if self.subTopic == "/diagnostics":
                self.subTopic = "/diagnostics_agg"
            elif self.subTopic == "/diagnostics_agg":
                self.subTopic = "/diagnostics" 
            
            self.diagnostics = DiagArray(self.subTopic)
            self.diagSub.unregister()
            self.diagSub = rospy.Subscriber(self.subTopic, 
                                            DiagnosticArray,
                                            self.diag_cb)
            self.setControls()
            self.paused = False
            self.display([osc.Message("/diagnostics/pause",0.0)])
            
    def pause_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        if value[0] == 0.0:
            self.paused = False
            self.display([osc.Message("/diagnostics/pause",0.0)])
            self.display([osc.Message("/diagnostics/pausetime","")])
        if value[0] == 1.0:
            self.paused = rospy.Time.now()
            self.display([osc.Message("/diagnostics/pause",1.0)])
            
        
        