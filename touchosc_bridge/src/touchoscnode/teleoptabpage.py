import roslib; roslib.load_manifest('touchosc_bridge')

import rospy

from txosc import osc
from txosc import dispatch
from txosc import async

from geometry_msgs.msg import Twist
import touchosc_msgs.msg

from abstracttabpage import AbstractTabpageHandler
from std_msgs.msg import String

from twisted.internet import reactor

class TeleopTabpageHandler(AbstractTabpageHandler):
    def __init__(self, nodeName, tabpageName, tabpageAlias, 
                 max_vx = 0.6,max_vy = 0.6,max_vw = 0.8,
                 max_run_vx = 1.0, max_run_vy = 1.0, max_run_vw = 1.0,
                 run = False):
        super(TeleopTabpageHandler, self).__init__(nodeName, tabpageName, tabpageAlias)
        
        self.max_vx = rospy.get_param("~max_vx",max_vx)
        self.max_vy = rospy.get_param("~max_vy",max_vy)
        self.max_vw = rospy.get_param("~max_vw",max_vw)
        self.max_run_vx = rospy.get_param("~max_run_vx",max_run_vx)
        self.max_run_vy = rospy.get_param("~max_run_vy",max_run_vy)
        self.max_run_vw = rospy.get_param("~max_run_vw",max_run_vw)
        self.running = rospy.get_param("~run", run)
        self.minPublishFreq = rospy.get_param("~min_freq",10) 
        
        self.pub = rospy.Publisher("cmd_vel",Twist)
        
        self.addOscCallback('xy', self.xypad_callback)
        self.addOscCallback('w', self.w_callback)
        self.addOscCallback('control', self.control_callback)
        self.addOscCallback('turbo', self.turbo_callback)
        
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
        
        self.masterOsc = None
        reactor.callLater(1.0/self.minPublishFreq, self.publish_cmd)
        
    def initializeTabpage(self):
        rospy.loginfo("Teleop Initialized")
        messageList = []
        messageList.append(osc.Message("/ipod/teleop/xy",0.0,0.0))
        messageList.append(osc.Message("/ipod/teleop/w",0.0))
        messageList.append(osc.Message("/ipod/teleop/control",0.0))
        messageList.append(osc.Message("/ipod/teleop/master",""))
        messageList.append(osc.Message("/ipod/teleop/turbo",0.0))
        self.oscSendToAll(osc.Bundle(messageList))
        self.zero_command()
        
    def zero_command(self):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
        self.running = False    
        
    def publish_cmd(self):
        if self.masterOsc:
            # Periodically resend the master address and the button status,
            # with local feedback off, the device sometimes misses the message
            # the first time, and you can't exit control or running mode.
            message = osc.Message("/teleop/master",self.masterOsc[1])
            self.oscSendToAll(message)
            message = osc.Bundle([osc.Message("/teleop/control",1.0),
                                 osc.Message("/teleop/turbo",self.running)])
            self.oscSendToClient(message,self.masterOsc[0])        
        self.pub.publish(self.cmd)
        reactor.callLater(1.0/self.minPublishFreq, self.publish_cmd)
    
    def xypad_callback(self, addressList, valueList, sendAddress):
        if self.masterOsc:
            # Is this the current master?
            if sendAddress[0] == self.masterOsc[0][0]:
                if len(addressList) == 2:
                    message = osc.Message("/teleop/xy",*valueList)
                    self.oscSendToAllOtherActive(message, self.masterOsc[0])
                    vx = self.max_run_vx if self.running else self.max_vx
                    vy = self.max_run_vy if self.running else self.max_vy
                    self.cmd.linear.x = max(min(valueList[0] * vx,vx),-vx)
                    self.cmd.linear.y = max(min(valueList[1] * vy,vy),-vy)
                elif valueList[0] == 0.0:
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = 0.0
                self.pub.publish(self.cmd)
        
    def w_callback(self, addressList, valueList, sendAddress):
        if self.masterOsc:
            if sendAddress[0] == self.masterOsc[0][0]:
                if len(addressList) == 2:
                    message = osc.Message("/teleop/w",valueList[0])
                    self.oscSendToAllOtherActive(message, self.masterOsc[0])
                    vw = self.max_run_vw if self.running else self.max_vw
                    self.cmd.angular.z = max(min(valueList[0] * vw,vw),-vw)
                elif valueList[0] == 0.0:
                    self.cmd.angular.z = 0.0
                self.pub.publish(self.cmd)
        
    def control_callback(self, addressList, valueList, sendAddress):
        if len(addressList) == 2:
            for client in self.clients.iterkeys():
                if (client[0] == sendAddress[0] 
                        and self.masterOsc == None
                        and valueList[0] == 1.0):
                    self.masterOsc = [client, 
                                      str(self.clients[client].split(".")[0])]
                    message = osc.Message("/teleop/master",self.masterOsc[1])
                    self.oscSendToAll(message)
                    message = osc.Message("/teleop/control",1.0)
                    self.oscSendToClient(message,client)
                elif (client[0] == sendAddress[0]
                        and valueList[0] == 0.0):
                    self.masterOsc = None
                    self.initializeTabpage()
                    
    def turbo_callback(self, addressList, valueList, sendAddress):
        if self.masterOsc and len(addressList) == 2:
            if sendAddress[0] == self.masterOsc[0][0]:
                if len(addressList) == 2 and valueList[0] == 1.0:
                    message = osc.Message("/teleop/turbo",valueList[0])
                    self.oscSendToAll(message)
                    self.running = True
                elif valueList[0] == 0.0:
                    message = osc.Message("/teleop/turbo",valueList[0])
                    self.oscSendToAll(message)
                    self.running = False