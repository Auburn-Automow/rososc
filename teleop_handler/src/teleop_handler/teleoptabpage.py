import roslib; roslib.load_manifest('teleop_handler')
import rospy

from txosc import osc
from geometry_msgs.msg import Twist

from touchosc_bridge.abstracttabpage import AbstractTabpageHandler
from twisted.internet import reactor

import socket

class TeleopTabpageHandler(AbstractTabpageHandler):
    def __init__(self, touchosc_interface, handler_name, tabpage_names,
                 max_vx=0.6, max_vy=0.6, max_vw=0.8,
                 max_run_vx=1.0, max_run_vy=1.0, max_run_vw=1.0,
                 run=False):
        super(TeleopTabpageHandler, self).__init__(touchosc_interface,
                                                   handler_name,
                                                   tabpage_names)

        pref = "~" + self.handler_name + "/"
        self.max_vx = rospy.get_param(pref + "max_vx", max_vx)
        self.max_vy = rospy.get_param(pref + "max_vy", max_vy)
        self.max_vw = rospy.get_param(pref + "max_vw", max_vw)
        self.max_run_vx = rospy.get_param(pref + "max_run_vx", max_run_vx)
        self.max_run_vy = rospy.get_param(pref + "max_run_vy", max_run_vy)
        self.max_run_vw = rospy.get_param(pref + "max_run_vw", max_run_vw)
        self.running = rospy.get_param(pref + "run", run)
        self.minPublishFreq = rospy.get_param(pref + "min_freq", 10)

        self.pub = rospy.Publisher("cmd_vel", Twist)

        self.add_osc_callback('xy', self.xypad_callback)
        self.add_osc_callback('w', self.w_callback)
        self.add_osc_callback('control', self.control_callback)
        self.add_osc_callback('mapping', self.mapping_callback)
        self.add_osc_callback('turbo', self.turbo_callback)

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0

        self.holonomic = True
        self.masterOsc = None
        reactor.callLater(1.0 / self.minPublishFreq, self.publish_cmd)

    def initializeTabpage(self):
        self.zero_command()

    def zero_command(self):
            self.sendToAll(osc.Bundle([osc.Message('xy', 0.0, 0.0),
                                    osc.Message('w', 0.0),
                                    osc.Message('control', 0.0),
                                    osc.Message('mapping', 0.0),
                                    osc.Message('turbo', 0.0),
                                    osc.Message('mapping_label', "holonomic"),
                                    osc.Message('master', "")]))
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
            self.holonomic = True
            self.running = False

    def zero_xy_command(self):
        self.sendToAll(osc.Message('xy', 0.0, 0.0))
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0

    def zero_w_command(self):
        self.sendToAll(osc.Message('w', 0.0))
        self.cmd.angular.z = 0.0

    def publish_cmd(self):
        if self.masterOsc and self.masterOsc not in self.activeClients:
                self.__oscSendToAll(osc.Bundle([osc.Message('control', 0.0),
                                           osc.Message('master', '')]))
                self.masterOsc = None
        elif self.masterOsc:
            self.sendToClient(osc.Message('control', 1.0), self.masterOsc)
        self.pub.publish(self.cmd)
        reactor.callLater(1.0 / self.minPublishFreq, self.publish_cmd)

    def xypad_callback(self, addressList, valueList, sendAddress):
        if sendAddress[0] not in self.activeClients:
                self.activeClients[sendAddress[0]] = addressList[0]
        if sendAddress[0] == self.masterOsc and len(addressList) == 2:
            self.sendToAllOtherActive(osc.Message('xy', *valueList),
                                      self.masterOsc)
            vx = self.max_run_vx if self.running else self.max_vx
            vy = self.max_run_vy if self.running else self.max_vy
            self.cmd.linear.x = max(min(valueList[0] * vx, vx), -vx)
            if self.holonomic:
                self.cmd.linear.y = max(min(valueList[1] * -vy, vy), -vy)
            else:
                self.cmd.angular.z = max(min(valueList[1] * -vy, vy), -vy)
        elif sendAddress[0] == self.masterOsc and valueList[0] == 0.0:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            if not self.holonomic:
                self.cmd.angular.z = 0.0
            self.zero_xy_command()
        self.pub.publish(self.cmd)

    def w_callback(self, addressList, valueList, sendAddress):
        if not self.holonomic:
            return
        if sendAddress[0] not in self.activeClients:
                self.activeClients[sendAddress[0]] = addressList[0]
        if sendAddress[0] == self.masterOsc and len(addressList) == 2:
            self.sendToAllOtherActive(osc.Message('w', *valueList),
                                      self.masterOsc)
            vw = self.max_run_vw if self.running else self.max_vw
            self.cmd.angular.z = max(min(valueList[0] * -vw, vw), -vw)
        elif sendAddress[0] == self.masterOsc and valueList[0] == 0.0:
            self.cmd.angular.z = 0.0
            self.zero_w_command()
        self.pub.publish(self.cmd)

    def mapping_callback(self, addressList, valueList, sendAddress):
        if len(addressList) == 2:
            self.holonomic = not self.holonomic
            self.sendToAll(osc.Message('w/visible', int(self.holonomic)))
            if self.holonomic:
                self.sendToAll(osc.Message('mapping_label', "Holonomic"))
            else:
                self.sendToAll(osc.Message('mapping_label', "Differential"))

    def control_callback(self, addressList, valueList, sendAddress):
        if len(addressList) == 2:
            if sendAddress[0] not in self.activeClients:
                self.activeClients[sendAddress[0]] = addressList[0]
                if not self.masterOsc:
                    name = self.masterOsc
                    self.sendToClient(osc.Message('control', 1.0), self.masterOsc)
                    self.sendToAll(osc.Message('master', name))
            elif not self.masterOsc and valueList[0] == 1.0:
                self.masterOsc = sendAddress[0]
                name = self.masterOsc
                self.sendToClient(osc.Message('control', 1.0), self.masterOsc)
                self.sendToAll(osc.Message('master', name))
            elif valueList[0] == 0.0:
                self.sendToAll(osc.Message('control', 0.0))
                self.sendToAll(osc.Message('master', ''))
                self.masterOsc = None

    def turbo_callback(self, addressList, valueList, sendAddress):
        if sendAddress[0] not in self.activeClients:
                self.activeClients[sendAddress[0]] = addressList[0]
        if sendAddress[0] == self.masterOsc and len(addressList) == 2:
            if valueList[0] == 1.0:
                message = osc.Message("turbo", valueList[0])
                self.sendToAll(message)
                self.running = True
            elif valueList[0] == 0.0:
                message = osc.Message("turbo", valueList[0])
                self.sendToAll(message)
                self.running = False

    def tabpageClosedCallback(self, client, tabpage):
        if client[0] in self.activeClients:
            del self.activeClients[client[0]]
        if client[0] == self.masterOsc:
            self.sendToAll(osc.Bundle([osc.Message('control', 0.0),
                                       osc.Message('master', '')]))
            self.masterOsc = None


