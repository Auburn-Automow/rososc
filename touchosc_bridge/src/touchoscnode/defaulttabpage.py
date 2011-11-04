import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch

from abstracttabpage import AbstractTabpageHandler
import pytouchosc
import touchosc_msgs.msg
import rospy

class DefaultTabpageHandler(AbstractTabpageHandler):
    def __init__(self, nodeName, layout, tabpageName):
        super(DefaultTabpageHandler, self).__init__(nodeName, tabpageName)
        
        self.layout = layout
        self.tabpage = self.layout.getTabpage(self.tabpageName)
        self.osc_node = dispatch.AddressNode(self.tabpageName)
        self.messageDict = self.tabpage.getMessages()
        self.osc_nodes = {}
        
        rosPrefix = self.nodeName + '/' + self.tabpageName + '/'
        
        for control in self.tabpage.iterchildren():
            controlType = type(control)
            if controlType is pytouchosc.controls.Button or \
                controlType is pytouchosc.controls.Dial:
                msgType = touchosc_msgs.msg.ScalableControl
                ros_cb = self.scalableControl_ros_cb
                osc_cb = self.scalableControl_osc_cb
            elif controlType is pytouchosc.controls.LED:
                msgType = touchosc_msgs.msg.ScalableControl
                ros_cb = self.scalableControl_ros_cb
                osc_cb = None
            elif controlType is pytouchosc.controls.Time or \
                controlType is pytouchosc.controls.TextField:
                msgType = touchosc_msgs.msg.TouchOSC_Common
                ros_cb = self.common_ros_cb
                osc_cb = None
            elif controlType is pytouchosc.controls.Label:
                msgType = touchosc_msgs.msg.Label
                ros_cb = self.label_ros_cb
                osc_cb = None
            elif controlType is pytouchosc.controls.Encoder:
                msgType = touchosc_msgs.msg.ScalableControl
                ros_cb = self.common_ros_cb
                osc_cb = self.scalableControl_osc_cb
            elif controlType is pytouchosc.controls.MultiButton:
                msgType = touchosc_msgs.msg.MultiButton
                ros_cb = self.multibutton_ros_cb
                osc_cb = self.multibutton_osc_cb
            elif controlType is pytouchosc.controls.MultiDial:
                msgType = touchosc_msgs.msg.MultiFader
                ros_cb = self.multifader_ros_cb
                osc_cb = self.multifader_osc_cb
            elif controlType is pytouchosc.controls.XYPad:
                msgType = touchosc_msgs.msg.XYPad
                ros_cb = self.xypad_ros_cb
                osc_cb = self.xypad_osc_cb
            elif controlType is pytouchosc.controls.MultiXYPad:
                msgType = touchosc_msgs.msg.MultiXYPad
                ros_cb = self.common_ros_cb
                osc_cb = self.multixypad_osc_cb
            else:
                msgType = None
                ros_cb = None
                osc_cb = None
            
            if msgType is not None:
                if ros_cb is not None:
                    self.ros_subscribers[control.name] = rospy.Subscriber(rosPrefix + control.name + '_sub',
                                                                          msgType,
                                                                          ros_cb)
                if osc_cb is not None:
                    self.ros_publishers[control.name] = rospy.Publisher(rosPrefix + control.name + '_pub',
                                                                        msgType)
                    self.osc_nodes[control.name] = dispatch.AddressNode(control.name)
                    self.osc_nodes[control.name].addCallback("*", osc_cb)
                    self.osc_nodes[control.name].addCallback("/*", osc_cb)
                    self.osc_nodes[control.name].addCallback("/*/*", osc_cb)
                    self.osc_node.addNode(control.name, self.osc_nodes[control.name])
    
    def common_ros_cb(self, msg):
        if type(msg) is not touchosc_msgs.msg.TouchOSC_Common:
            msg = msg.common
        ctDict = self.messageDict[msg.name]
        address = '/' + msg.tabpage + '/' + msg.name
        if msg.color != ctDict['color']:
            print "Sending color to %s"%address
            self.osc_send(osc.Message(address + '/color', msg.color))
            ctDict['color'] = msg.color
#        if msg.x != int(ctDict['position']['x']):
#            self.osc_send(osc.Message(address + '/position/x', msg.x))
#        if msg.y != int(ctDict['position']['y']):
#            self.osc_send(osc.Message(address + '/position/x', msg.y))
#        if msg.width != int(ctDict['size']['w']):
#            self.osc_send(osc.Message(address+ '/size/w', msg.width))
#        if msg.height != int(ctDict['size']['h']):
#            self.osc_send(osc.Message(address+ '/size/h', msg.height))
#        if msg.visibility != bool(ctDict['visibility']):
#            self.osc_send(osc.Message(address+'/visibility', msg.visibility))
    
    def scalableControl_ros_cb(self, msg):
        self.common_ros_cb(msg.common)
        ctDict = self.messageDict[msg.common.name]
        address = '/' + msg.common.tabpage + '/' + msg.common.name
        if msg.value != ctDict[None]:
            ctDict[None] = msg.value
            self.osc_send(osc.Message(address, msg.value))

    def label_ros_cb(self, msg):
        self.common_ros_cb(msg.common)
        ctDict = self.messageDict[msg.common.name]
        address = '/' + msg.common.tabpage + '/' + msg.common.name
        if str(msg.value) != ctDict['text']:
            ctDict['text'] = str(msg.value)
            self.osc_send(osc.Message(address, ctDict['text']))
            
    def multibutton_ros_cb(self, msg):
        self.common_ros_cb(msg.common)
        ctDict = self.messageDict[msg.common.name]
        address = '/' + msg.common.tabpage + '/' + msg.common.name
        if list(msg.values) != ctDict[None]:
            ctDict[None] = list(msg.values)
            message = osc.Message(address,*msg.values)
            self.osc_send(message)
            
    def multifader_ros_cb(self, msg):
        self.common_ros_cb(msg.common)
        ctDict = self.messageDict[msg.common.name]
        address = '/' + msg.common.tabpage + '/' + msg.common.name
        
        if list(msg.values) != ctDict[None]:
            ctDict[None] = list(msg.values)
            message = osc.Message(address,*ctDict[None])
            self.osc_send(message)
            
    def xypad_ros_cb(self, msg):
        self.common_ros_cb(msg.common)
        ctDict = self.messageDict[msg.common.name]
        address = '/' + msg.common.tabpage + '/' + msg.common.name
        
        if ctDict[None][0] != msg.x or ctDict[None][1] != msg.y:
            ctDict[None] = [msg.x, msg.y]
            message = osc.Message(address,*ctDict[None])
            self.osc_send(message)
    
    def scalableControl_osc_cb(self, message, address):
        tabpageName = osc.getAddressParts(message.address)[0]
        controlName = osc.getAddressParts(message.address)[1]
        ctDict = self.messageDict[controlName]
        
        value = message.getValues()
        if len(osc.getAddressParts(message.address)) == 3:
            ctDict['z'] = bool(value[0])
        else:
            ctDict[None] = float(value[0])
            
        msg = touchosc_msgs.msg.ScalableControl()
        msg.header.stamp = rospy.Time.now()
        msg.common = self.populate_common(tabpageName, controlName)
        msg.z = ctDict['z']
        msg.value = ctDict[None]
        self.ros_publishers[controlName].publish(msg)

    def multibutton_osc_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        tabpageName = addParts[0]
        controlName = addParts[1]
        ctDict = self.messageDict[controlName]
        
        if len(addParts) == 3:
            ctDict['z'] = bool(value[0])
        else:
            x = int(addParts[2]) - 1
            y = int(addParts[3]) - 1
            ctDict[None][y + x*ctDict['dim_y']] = value[0]
        
        value = message.getValues()
        msg = touchosc_msgs.msg.MultiButton()
        msg.header.stamp = rospy.Time.now()
        msg.common = self.populate_common(tabpageName, controlName)
        msg.dimension = [ctDict['dim_x'], ctDict['dim_y']]
        msg.z = ctDict['z']
        msg.values = ctDict[None]
        self.ros_publishers[controlName].publish(msg)
    
    def multifader_osc_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        tabpageName = addParts[0]
        controlName = addParts[1]
        ctDict = self.messageDict[controlName]
        if addParts[2] is 'z':
            ctDict['z'] = bool(value[0])
        else:
            pos = int(addParts[2]) - 1
            ctDict[None][pos] = value[0]  
        msg = touchosc_msgs.msg.MultiFader()
        msg.header.stamp = rospy.Time.now()
        msg.common = self.populate_common(tabpageName, controlName)
        msg.dimension = ctDict['number']
        msg.z = ctDict['z']
        msg.values = ctDict[None]
        self.ros_publishers[controlName].publish(msg)
    
    def multixypad_osc_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        tabpageName = addParts[0]
        controlName = addParts[1]
        ctDict = self.messageDict[controlName]
        touchNumber = int(addParts[2])
        
        if len(addParts) == 4:
            ctDict['z'][touchNumber - 1] = bool(value[0])
            if not bool(value[0]):
                ctDict['x'][touchNumber - 1] = 0.0
                ctDict['y'][touchNumber - 1] = 0.0
        else:
            ctDict['x'][touchNumber - 1] = value[0]
            ctDict['y'][touchNumber - 1] = value[1]
        
        msg = touchosc_msgs.msg.MultiXYPad()
        msg.header.stamp = rospy.Time.now()
        msg.common = self.populate_common(tabpageName, controlName)
        msg.z = ctDict['z']        
        msg.x = ctDict['x']
        msg.y = ctDict['y']
        self.ros_publishers[controlName].publish(msg)
            
    def xypad_osc_cb(self, message, address):
        addParts = osc.getAddressParts(message.address)
        value = message.getValues()
        
        tabpageName = addParts[0]
        controlName = addParts[1]
        ctDict = self.messageDict[controlName]
 
        if len(addParts) == 3:
            ctDict['z'] = bool(value[0])
        else:
            ctDict[None] = value
 
        msg = touchosc_msgs.msg.XYPad()
        msg.header.stamp = rospy.Time.now()
        msg.common = self.populate_common(tabpageName, controlName)
        msg.x = ctDict[None][0]
        msg.y = ctDict[None][1]
        msg.z = ctDict['z']
        self.ros_publishers[controlName].publish(msg)
        
    def populate_common(self, tabpageName, controlName):
        commonMsg = touchosc_msgs.msg.TouchOSC_Common()
        ctDict = self.messageDict[controlName]
        commonMsg.tabpage = tabpageName
        commonMsg.name = controlName
        commonMsg.color = ctDict['color']
        commonMsg.x = int(ctDict['position']['x'])
        commonMsg.y = int(ctDict['position']['y'])
        commonMsg.width = int(ctDict['size']['w'])
        commonMsg.height = int(ctDict['size']['h'])
        commonMsg.visibility = ctDict['visibility']
        return commonMsg
