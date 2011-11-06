import roslib; roslib.load_manifest('diagnostics_handler')
import rospy

from txosc import osc
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from touchoscnode import AbstractTabpageHandler

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
    
    def getDisplayList(self):
        displayList = sorted(self.diagDict.keys())
        messages = [self.diagDict[x] for x in displayList]
        return [(msg.getName(),msg.getColor()) for msg in messages]


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
        
        self.diagOffset = 0
        self.detailOffset = 0
        
        if self.clientType.lower() == "ipad":
            self.lengthDiagnosticList = 16
            self.lengthDetailDiagnosticList = 8
        elif self.clientType.lower() == "ipod":
            self.lengthDiagnosticList = 10
            self.lengthDetailDiagnosticList = 6
        else:
            raise ValueError("Client type %s is not supported"%self.clientType)
        self.clientTopic = clientTopic
        
        self.msgList = None
        self.paused = False
    
    def setTopic(self, newTopic):
        self.clientTopic = newTopic
        if self.clientTopic == '/diagnostics':
            self.msgList = self.diagArray
        else:
            self.msgList = self.diagAggArray
    
    def getMaxMinDiagOffset(self):
        maxOffset = self.msgList.getLength() - self.lengthDiagnosticList
        maxOffset = 0 if maxOffset < 0 else maxOffset
        return (maxOffset, 0)
    
    def getMaxMinDetailOffset(self):
        pass
        
    def setDiagOffset(self, offset):
        self.diagOffset = offset
    
    def getTopic(self):
        return self.clientTopic
    
    def setMessageDb(self, diagArray, diagAggArray):
        self.diagArray = diagArray
        self.diagAggArray = diagAggArray
        self.msgList = diagArray
    
    def updateDisplay(self, nofader = False):
        it = 1
        toDisplay = []
        displayList = self.msgList.getDisplayList()
        for (name, color) in displayList[self.diagOffset:]:
            toDisplay.append(osc.Message("dled%i/color"%it, color))
            toDisplay.append(osc.Message("dled%i"%it,1.0))
            toDisplay.append(osc.Message("dlabel%i"%it,name))
            it+=1
            if it > self.lengthDiagnosticList: break;
        while it <= self.lengthDiagnosticList:
            toDisplay.append(osc.Message("dled%i/color"%it, 'gray'))
            toDisplay.append(osc.Message("dled%i"%it,0.0))
            toDisplay.append(osc.Message("dlabel%i"%it,''))
            it +=1
        toDisplay.append(osc.Message("statusled/color", self.msgList.getSystemStatus()))
        toDisplay.append(osc.Message("statusled",1.0))
        if not nofader:
            if len(displayList) <= self.lengthDiagnosticList:
                value = 1.0
            else:
                value = self.diagOffset/float(len(displayList) - self.lengthDiagnosticList)
            toDisplay.append(osc.Message("dfader",value))
        return toDisplay

    def clearDisplay(self):
        it = 1
        toDisplay = []
        while it <= self.lengthDiagnosticList:
            toDisplay.append(osc.Message("dled%i/color"%it, 'gray'))
            toDisplay.append(osc.Message("dled%i"%it,0.0))
            toDisplay.append(osc.Message("dlabel%i"%it,''))
            it+=1
        toDisplay.append(osc.Message("/diagnostics/dfader",0.0))
        toDisplay.append(osc.Message("/diagnostics/statusled",0.0))
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
    def __init__(self, nodeName, tabpageName, tabpageAlias):
        super(DiagnosticsTabpageHandler, self).__init__(nodeName, tabpageName, tabpageAlias)

        self.startTopic = rospy.get_param("~" + tabpageName + "/start_topic", "/diagnostics")

        #=======================================================================
        # self.addOscCallback('darray',self.darray_cb)
        # self.osc_node.addCallback('/kvup',self.kv_updown_cb)
        # self.osc_node.addCallback('/kvdown',self.kv_updown_cb)
        # self.osc_node.addCallback('/kvfader',self.kv_updown_cb)
        self.addOscCallback('dup',self.d_updown_cb)
        self.addOscCallback('ddown',self.d_updown_cb)
        self.addOscCallback('dfader',self.d_updown_cb)
        self.addOscCallback('diagsw',self.diagsw_cb)
        # self.osc_node.addCallback('/pause',self.pause_cb)
        #=======================================================================
        
        self.diagArray = DiagArray("/diagnostics")
        self.diagAggArray = DiagArray("/diagnostics_agg")
        self.diagClients = {}

    
    def initializeTabpage(self):
        rospy.loginfo("Diagnostics Tabpage Initialized")
        rospy.loginfo("\t Subscribing to: /diagnostics")
        rospy.loginfo("\t Subscribing to: /diagnostics_agg")
        
        self.diagSub = rospy.Subscriber("/diagnostics", 
                                        DiagnosticArray,
                                        self.diag_cb)
        self.diagAggSub = rospy.Subscriber("/diagnostics_agg",
                                           DiagnosticArray,
                                           self.diag_agg_cb)
        
        self.sendToAll(osc.Bundle([osc.Message('diaglbl','Start'),
                                   osc.Message('statusled',1.0)]))
        it = 1
        toDisplay = []
        while it <= 17:
            toDisplay.append(osc.Message("dled%i/color"%it, 'gray'))
            toDisplay.append(osc.Message("dled%i"%it,0.0))
            toDisplay.append(osc.Message("dlabel%i"%it,''))
            it+=1
        toDisplay.append(osc.Message("statusled/color","red"))
        toDisplay.append(osc.Message("statusled",1.0))
        self.sendToAll(osc.Bundle(toDisplay))
        
    def diag_cb(self, msg):
        self.diagArray.addMessage(msg)
        for address, client in self.diagClients.iteritems():
            toSend = []
            if not client.paused and client.getTopic() == '/diagnostics':
                toSend.extend(client.updateDisplay(True))
                self.sendToClient(osc.Bundle(toSend), address)
                
    def diag_agg_cb(self, msg):
        self.diagAggArray.addMessage(msg)
        for address, client in self.diagClients.iteritems():
            toSend = []
            if not client.paused and client.getTopic() == '/diagnostics_agg':
                toSend.extend(client.updateDisplay(True))
                self.sendToClient(osc.Bundle(toSend), address)
                
    def d_updown_cb(self, addressList, valueList, sendAddress):
        client = self.diagClients[sendAddress[0]]
        offset = client.diagOffset
        
        maxOffset, minOffset = client.getMaxMinDiagOffset()
        if addressList[1] == 'ddown':
            if offset < maxOffset:
                client.setDiagOffset(offset + 1)
            nofader = False
        elif addressList[1] == 'dup':
            if offset > minOffset:
                client.setDiagOffset(offset - 1)
            nofader = False
        elif addressList[1] == 'dfader':
            client.setDiagOffset(int(round(valueList[0]*maxOffset)))
            nofader = True
        self.sendToClient(osc.Bundle(client.updateDisplay(nofader)), 
                          sendAddress[0])
            
    def diagsw_cb(self, addressList, valueList, sendAddress):
        if len(addressList) == 2 and valueList[0] == 1.0:
            if sendAddress[0] not in self.diagClients:
                self.activeClients[sendAddress[0]] = addressList[0]
                if addressList[0].find('ipod') != -1:
                    clientType = 'ipod'
                elif addressList[0].find('ipad') != -1:
                    clientType = 'ipad'
                else:
                    rospy.logerr("Unknown client type in diagnostics handler!")
                self.diagClients[sendAddress[0]] = DiagnosticsClient(sendAddress[0],
                                                                     clientType,
                                                                     self.startTopic)
                self.diagClients[sendAddress[0]].setMessageDb(self.diagArray, 
                                                              self.diagAggArray)
                client = self.diagClients[sendAddress[0]]
            else:
                try:
                    client = self.diagClients[sendAddress[0]]
                except KeyError:
                    return
                if client.getTopic() == '/diagnostics':
                    client.setTopic('/diagnostics_agg')
                elif client.getTopic() == '/diagnostics_agg':
                    client.setTopic('/diagnostics')
            
            if client.clientType == 'ipod':
                val = 'Raw' if client.getTopic() == '/diagnostics' else 'Aggregate'
            else:
                val = client.getTopic()
                    
            self.sendToClient(osc.Message('diaglbl',val),sendAddress[0])
            
    def updateDiagnostics(self):
        tabpageStatus = DiagnosticStatus()
        tabpageStatus.level = tabpageStatus.OK
        tabpageStatus.name = self.tabpageName + " Handler"
        tabpageStatus.hardware_id = self.nodeName
        tabpageStatus.message = "OK"
        tabpageStatus.values = []
        for clientName, clientObj in self.diagClients.iteritems():
            tabpageStatus.values.append(KeyValue(key=clientName, value=clientObj.getTopic()))
            tabpageStatus.values.append(KeyValue(key=clientName, value=clientObj.clientType))
        return tabpageStatus
            
        
