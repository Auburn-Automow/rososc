import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch

from abstracttabpage import AbstractTabpageHandler
import pytouchosc
import touchosc_msgs.msg
import rospy

class DefaultTabpageHandler(AbstractTabpageHandler):
    def __init__(self, nodeName, tabpageName, tabpage):
        super(DefaultTabpageHandler, self).__init__(nodeName, tabpageName)
        
        self.tabpage = tabpage
        self.messageDict = self.tabpage.getMessages()
        
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
                self.ros_subscribers[control.name] = rospy.Subscriber(rosPrefix + control.name, msgType,
                                                                          ros_cb)
                self.ros_publishers[control.name] = rospy.Publisher(rosPrefix + control.name, msgType)
                
                if osc_cb:
                    self.addOscCallback(control.name, osc_cb)    
    
    def osc_populate_common(self, msg):
        topic = msg._connection_header['topic'].split('/')
        tabpageName = topic[2]
        controlName = topic[3]
        toSend = []
        ctDict = self.messageDict[controlName]
        if msg.common.color != '':
            toSend.append(osc.Message('/'.join([controlName,'color']), 
                                      msg.common.color))
            ctDict['color'] = msg.common.color
        if msg.common.x != 0 and msg.common.x != int(ctDict['position']['x']):
            toSend.append(osc.Message('/'.join([controlName,'position/x']), 
                                      msg.common.x))
            ctDict['position']['x'] = msg.common.x
        if msg.common.y != 0 and msg.common.y != int(ctDict['position']['y']):
            toSend.append(osc.Message('/'.join([controlName,'position/y']), 
                                      msg.common.y))
            ctDict['position']['y'] = msg.common.y
        if msg.common.width != 0 and msg.common.width != int(ctDict['size']['w']):
            toSend.append(osc.Message('/'.join([controlName,'size/w']), 
                                      msg.common.width))
            ctDict['size']['w'] = msg.common.width
        if msg.common.height != 0 and msg.common.height != int(ctDict['size']['h']):
            toSend.append(osc.Message('/'.join([controlName,'size/h']), 
                                      msg.common.height))
            ctDict['size']['h'] = msg.common.height
        if msg.common.visible != '':
            send = 0 if msg.common.visible.lower() == 'false' else 1
            if bool(send) != bool(ctDict['visibility']):
                toSend.append(osc.Message('/'.join([controlName,'visible']),
                                          send))
                ctDict['visibility'] = bool(send)
        return (tabpageName, controlName, ctDict, toSend)
    
    def common_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))
    
    def scalableControl_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if msg.value != ctDict[None]:
                ctDict[None] = msg.value
                toSend.append(osc.Message(control, msg.value))
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))

    def label_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if msg.value != ctDict['text']:
                ctDict['text'] = msg.value
                toSend.append(osc.Message(control, msg.value))
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))
            
    def multibutton_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if list(msg.values) != ctDict[None]:
                ctDict[None] = list(msg.values)
                toSend.append(osc.Message(control, *msg.value))
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))
            
    def multifader_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if list(msg.values) != ctDict[None]:
                ctDict[None] = list(msg.values)
                toSend.append(osc.Message(control,*ctDict[None]))
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))
            
    def xypad_ros_cb(self, msg):
        if msg._connection_header['callerid'] != self.nodeName:
            (tabpage, control, ctDict, toSend) = self.osc_populate_common(msg)
            if [msg.x, msg.y] != ctDict[None]:
                ctDict[None] = [msg.x, msg.y]
                toSend.append(osc.Message(control,*ctDict[None]))
            if msg.header.frame_id != '':
                self.sendToClient(osc.Bundle(toSend),msg.header.frame_id)
            else:
                self.sendToAll(osc.Bundle(toSend))
    
    def scalableControl_osc_cb(self, addressList, valueList, sendAddress):
        tabpageName = addressList[0]
        controlName = addressList[1]
        ctDict = self.messageDict[controlName]
        
        if len(addressList) == 3:
            ctDict['z'] = bool(valueList[0])
        else:
            ctDict[None] = float(valueList[0])
            
        msg = touchosc_msgs.msg.ScalableControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = sendAddress[0]
        msg.common = self.populate_common(tabpageName, controlName)
        msg.range = [ctDict['scalef'], ctDict['scalet']]
        msg.z = ctDict['z']
        msg.value = ctDict[None]
        self.ros_publishers[controlName].publish(msg)

    def multibutton_osc_cb(self, addressList, valueList, sendAddress):        
        tabpageName = addressList[0]
        controlName = addressList[1]
        ctDict = self.messageDict[controlName]
        
        if len(addressList) == 3:
            ctDict['z'] = bool(valueList[0])
        else:
            x = int(addressList[2]) - 1
            y = int(addressList[3]) - 1
            ctDict[None][y + x*ctDict['dim_y']] = valueList[0]
        msg = touchosc_msgs.msg.MultiButton()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = sendAddress[0]
        msg.common = self.populate_common(tabpageName, controlName)
        msg.dimension = [ctDict['dim_x'], ctDict['dim_y']]
        msg.range = [ctDict['scalef'], ctDict['scalet']]
        msg.z = ctDict['z']
        msg.values = ctDict[None]
        self.ros_publishers[controlName].publish(msg)
    
    def multifader_osc_cb(self, addressList, valueList, sendAddress):
        tabpageName = addressList[0]
        controlName = addressList[1]
        ctDict = self.messageDict[controlName]
        if len(addressList) > 2:
            if addressList[2] is 'z':
                ctDict['z'] = bool(valueList[0])
            else:
                pos = int(addressList[2]) - 1
                ctDict[None][pos] = valueList[0]
        msg = touchosc_msgs.msg.MultiFader()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = sendAddress[0]
        msg.common = self.populate_common(tabpageName, controlName)
        msg.dimension = ctDict['number']
        msg.range = [ctDict['scalef'], ctDict['scalet']]
        msg.z = ctDict['z']
        msg.values = ctDict[None]
        self.ros_publishers[controlName].publish(msg)
    
    def multixypad_osc_cb(self, addressList, valueList, sendAddress):
        tabpageName = addressList[0]
        controlName = addressList[1]
        ctDict = self.messageDict[controlName]
        touchNumber = int(addressList[2])
        
        if len(addressList) == 4:
            ctDict['z'][touchNumber - 1] = bool(valueList[0])
            if not bool(valueList[0]):
                ctDict['x'][touchNumber - 1] = 0.0
                ctDict['y'][touchNumber - 1] = 0.0
        else:
            ctDict['x'][touchNumber - 1] = valueList[0]
            ctDict['y'][touchNumber - 1] = valueList[1]
            
        msg = touchosc_msgs.msg.MultiXYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = sendAddress[0]
        msg.common = self.populate_common(tabpageName, controlName)
        msg.range = [ctDict['scalef'], ctDict['scalet']]
        msg.z = ctDict['z']        
        msg.x = ctDict['x']
        msg.y = ctDict['y']
        self.ros_publishers[controlName].publish(msg)
            
    def xypad_osc_cb(self, addressList, valueList, sendAddress):
        tabpageName = addressList[0]
        controlName = addressList[1]
        ctDict = self.messageDict[controlName]
 
        if len(addressList) == 3:
            ctDict['z'] = bool(valueList[0])
        else:
            ctDict[None] = valueList
 
        msg = touchosc_msgs.msg.XYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = sendAddress[0]
        msg.common = self.populate_common(tabpageName, controlName)
        msg.range = [ctDict['scalef'], ctDict['scalet']]
        msg.x = ctDict[None][0]
        msg.y = ctDict[None][1]
        msg.z = ctDict['z']
        self.ros_publishers[controlName].publish(msg)
        
    def populate_common(self, tabpageName, controlName):
        commonMsg = touchosc_msgs.msg.CommonProperties()
        ctDict = self.messageDict[controlName]
        commonMsg.tabpage = tabpageName
        commonMsg.name = controlName
        commonMsg.color = ctDict['color']
        commonMsg.x = int(ctDict['position']['x'])
        commonMsg.y = int(ctDict['position']['y'])
        commonMsg.width = int(ctDict['size']['w'])
        commonMsg.height = int(ctDict['size']['h'])
        commonMsg.visible = str(ctDict['visibility'])
        return commonMsg
