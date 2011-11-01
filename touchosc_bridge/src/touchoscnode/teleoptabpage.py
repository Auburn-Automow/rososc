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
    def __init__(self, nodeName,max_vx = 0.6,max_vy = 0.6,max_vw = 0.8,
                 max_run_vx = 1.0, max_run_vy = 1.0, max_run_vw = 1.0,
                 run = False):
        super(TeleopTabpageHandler, self).__init__(nodeName,"teleop")
        
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
        
    def setControls(self):
        messageList = []
        messageList.append(osc.Message("/teleop/xy",0.0,0.0))
        messageList.append(osc.Message("/teleop/w",0.0))
        messageList.append(osc.Message("/teleop/control",0.0))
        messageList.append(osc.Message("/teleop/master",""))
        self.oscSendToAll(osc.Bundle(messageList))
        self.zero_command()
        
    def publish_cmd(self):
        self.pub.publish(self.cmd)
        reactor.callLater(1.0/self.minPublishFreq, self.publish_cmd)
    
    def xypad_callback(self, message, address):
        addParts = osc.getAddressParts(message.address)
        values = message.getValues()
        if self.masterOsc:
            # Is this the current master?
            if address[0] == self.masterOsc[1]:
                if len(addParts) == 2:
                    message = osc.Message("/teleop/xy",values[0], values[1])
                    self.oscSendToAllOthers(message, self.masterOsc[0])
                    vx = self.max_run_vx if self.running else self.max_vx
                    vy = self.max_run_vy if self.running else self.max_vy
                    self.cmd.linear.x = max(min(values[0] * vx,vx),-vx)
                    self.cmd.linear.y = max(min(values[1] * vy,vy),-vy)
                elif values[0] == 0.0:
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = 0.0
                self.pub.publish(self.cmd)
        
    def w_callback(self, message, address):
        addParts = osc.getAddressParts(message.address)
        values = message.getValues()
        if self.masterOsc:
            # Is this the current master?
            if address[0] == self.masterOsc[1]:
                if len(addParts) == 2:
                    message = osc.Message("/teleop/w",values[0])
                    self.oscSendToAllOthers(message, self.masterOsc[0])
                    vw = self.max_run_vw if self.running else self.max_vw
                    self.cmd.angular.z = max(min(values[0] * vw,vw),-vw)
                elif values[0] == 0.0:
                    self.cmd.angular.z = 0.0
                self.pub.publish(self.cmd)
        
    def control_callback(self, message, address):
        addParts = osc.getAddressParts(message.address)
        values = message.getValues()
        if len(addParts) == 2:
            for clientName,clientAddress in self.clients.iteritems():
                if (clientAddress["ip"] == address[0] 
                        and self.masterOsc == None
                        and values[0] == 1.0):
                    self.masterOsc = [clientName, clientAddress["ip"]]
                    message = osc.Message("/teleop/master",
                                          str(clientName.split(".")[0]))
                    self.oscSendToAll(message)
                    message = osc.Message("/teleop/control",1.0)
                    self.oscSendToClient(message,clientName)
                elif(clientAddress["ip"] == address[0]
                        and values[0] == 0.0):
                    self.masterOsc = None
                    self.setControls()
                    
    def turbo_callback(self, message, address):
        addParts = osc.getAddressParts(message.address)
        values = message.getValues()
        if self.masterOsc and len(addParts) == 2:
            if address[0] == self.masterOsc[1]:
                if len(addParts) == 2 and values[0] == 1.0:
                    message = osc.Message("/teleop/turbo",values[0])
                    self.oscSendToAll(message)
                    self.running = True
                elif values[0] == 0.0:
                    message = osc.Message("/teleop/turbo",values[0])
                    self.oscSendToAll(message)
                    self.running = False
            
    def zero_command(self):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
                    