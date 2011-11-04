import roslib; roslib.load_manifest('touchosc_handlers')
import rospy

from txosc import osc
from geometry_msgs.msg import Twist

from touchosc_bridge import AbstractTabpageHandler
from twisted.internet import reactor

import socket

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
        self.sendToAll(osc.Bundle([osc.Message('xy',0.0,0.0),
                                   osc.Message('w',0.0),
                                   osc.Message('control',0.0),
                                   osc.Message('turbo', 0.0),
                                   osc.Message('master',"")]))
        self.zero_command()
        
    def zero_command(self):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
        self.running = False    
        
    def publish_cmd(self):
        if self.masterOsc:
            self.sendToClient(osc.Message('control',1.0), self.masterOsc)       
        self.pub.publish(self.cmd)
        reactor.callLater(1.0/self.minPublishFreq, self.publish_cmd)
    
    def xypad_callback(self, addressList, valueList, sendAddress):
        if sendAddress[0] == self.masterOsc and len(addressList) == 2:
            self.sendToAllOtherActive(osc.Message('xy', *valueList),
                                      self.masterOsc)
            vx = self.max_run_vx if self.running else self.max_vx
            vy = self.max_run_vy if self.running else self.max_vy
            self.cmd.linear.x = max(min(valueList[0] * vx,vx),-vx)
            self.cmd.linear.y = max(min(valueList[1] * vy,vy),-vy)
        elif sendAddress[0] == self.masterOsc and valueList[0] == 0.0:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
        self.pub.publish(self.cmd)

    def w_callback(self, addressList, valueList, sendAddress):
        if sendAddress[0] == self.masterOsc and len(addressList) == 2:
            self.sendToAllOtherActive(osc.Message('w', *valueList),
                                      self.masterOsc)
            vw = self.max_run_vw if self.running else self.max_vw
            self.cmd.angular.z = max(min(valueList[0] * vw,vw),-vw)
        elif sendAddress[0] == self.masterOsc and valueList[0] == 0.0:
            self.cmd.angular.z = 0.0
        self.pub.publish(self.cmd)
        
    def control_callback(self, addressList, valueList, sendAddress):
        if len(addressList) == 2:
            if sendAddress[0] not in self.activeClients:
                self.activeClients[sendAddress[0]] = addressList[0]
            if not self.masterOsc and valueList[0] == 1.0:
                self.masterOsc = sendAddress[0]
                name = socket.gethostbyaddr(self.masterOsc)
                name = name[0].split(".")[0]
                self.sendToClient(osc.Message('control',1.0), self.masterOsc)
                self.sendToAll(osc.Message('master',name))
            elif self.masterOsc and valueList[0] == 0.0:
                self.sendToClient(osc.Message('control',0.0), self.masterOsc)
                self.masterOsc = None
                self.sendToAll(osc.Message('master',''))
                    
    def turbo_callback(self, addressList, valueList, sendAddress):
        pass

        
        
