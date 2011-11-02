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
    
    def getLength(self):
        return len(self.kvDisplay)
    
            
class DiagArray(object):
    COLORS = {0: "green", 1: "yellow", 2: "red", 3: "gray"}
    def __init__(self, subTopic):
        self.diagDict = {}
        self.diagDisplay = []
        self.systemStatus = DiagnosticStatus.OK
        self.subTopic = subTopic # Should be "/diagnostics" or "/diagnostics_agg"
        self.expandName = None
        
    def getLength(self):
        return len(self.diagDisplay)
    
    def addMessage(self, msg):
        if msg is not None:
            for message in msg.status:
                # If the message has not been seen, add it.
                if not self.diagDict.has_key(message.name):
                    self.diagDict[message.name] = DiagMessage(message.name)
                self.diagDict[message.name].update(msg.header.stamp, message)
                if message.level != 3:
                    self.systemStatus = max(self.systemStatus,message.level)   
            self.diagDisplay = sorted(self.diagDict.keys())
        
    def getSystemStatus(self):
        return self.COLORS[self.systemStatus]


class DiagnosticsClient(object):
    def __init__(self, client, clientType, clientTopic):
        """
        Constructor
        
        @type client: C{list}
        @param client: (address, port) tuple for the client
        @type clientType: C{str}
        @param clientType: Type of the client "ipod" or "ipad"
        @type clientTopic: C{str}
        @param clientTopic: Topic of the client "/diagnostics" or "/diagnostics_agg"
        """
        self.client = client
        self.clientType = clientType
        
        if self.clientType == "ipad":
            self.lengthDiagnosticList = 16
            self.lengthDetailDiagnosticList = 8
        elif self.clientType == "ipod":
            self.lengthDiagnosticList = 0
            self.lengthDetailDiagnosticList = 0
        else:
            raise ValueError("Client type %s is not supported"%self.clientType)
        self.clientTopic = clientTopic
        
    def updateDisplay(self):
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
        while it <= self.lengthDiagnosticList:
            message=osc.Message("/diagnostics/dled%i/color"%it, 'gray')
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dled%i"%it,0.0)
            toDisplay.append(message)
            message=osc.Message("/diagnostics/dlabel%i"%it,'')
            toDisplay.append(message)
            it +=1
        return toDisplay

    def clearDisplay(self):
        it = 1
        toDisplay = []
        while it <= self.lengthDiagnosticList:
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
        return toDisplay

    def clearDetailedDisplay(self):
        it = 1
        toDisplay = []
        while it <= self.lengthDetailDiagnosticList:
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
        
        self.diagSub = rospy.Subscriber("/diagnostics", 
                                        DiagnosticArray,
                                        self.diag_cb)
        self.diagAggSub = rospy.Subscriber("/diagnostics_agg",
                                           DiagnosticArray,
                                           self.diag_cb)

        #=======================================================================
        # self.addOscCallback('darray',self.darray_cb)
        # self.osc_node.addCallback('/kvup',self.kv_updown_cb)
        # self.osc_node.addCallback('/kvdown',self.kv_updown_cb)
        # self.osc_node.addCallback('/kvfader',self.kv_updown_cb)
        # self.osc_node.addCallback('/dup',self.d_updown_cb)
        # self.osc_node.addCallback('/ddown',self.d_updown_cb)
        # self.osc_node.addCallback('/dfader',self.d_updown_cb)
        self.osc_node.addCallback('/diagsw',self.diagsw_cb)
        # self.osc_node.addCallback('/pause',self.pause_cb)
        #=======================================================================
        
        self.diagArray = DiagArray("/diagnostics")
        self.diagAggArray = DiagArray("/diagnostics_agg")
        
    def initializeTabpage(self):
        self.oscSendToAll(osc.Message("/diagnostics/diaglbl","Start"))
        self.oscSendToAll(osc.Message("/diagnostics/statusled",1.0))
        
    def diag_cb(self, msg):
        cHeader = msg._connection_header
        if cHeader['topic'] == '/diagnostics':
            self.diagArray.addMessage(msg)
        elif cHeader['topic'] == '/diagnostics_agg':
            self.diagAggArray.addMessage(msg)
        
    def diagsw_cb(self, addressList, valueList, sendAddress):
        print addressList
        print valueList
        print sendAddress
        