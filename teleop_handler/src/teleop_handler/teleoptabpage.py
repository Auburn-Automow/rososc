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
        self.master_osc = None
        reactor.callLater(1.0 / self.minPublishFreq, self.publish_cmd)
        self.active_clients = set()

    def initializeTabpage(self):
        self.zero_command()

    def zero_command(self):
        self.send(osc.Bundle([osc.Message('xy', 0.0, 0.0),
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
        self.send(osc.Message('xy', 0.0, 0.0))
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0

    def zero_w_command(self):
        self.send(osc.Message('w', 0.0))
        self.cmd.angular.z = 0.0

    def publish_cmd(self):
        if self.master_osc and self.master_osc not in self.active_clients:
            self.send(osc.Bundle([osc.Message('control', 0.0),
                                  osc.Message('master', '')]))
            self.master_osc = None
        elif self.master_osc:
            self.send(osc.Message('control', 1.0), clients=[self.master_osc])
        self.pub.publish(self.cmd)
        reactor.callLater(1.0 / self.minPublishFreq, self.publish_cmd)

    def xypad_callback(self, address_list, value_list, send_address):
        if send_address[0] not in self.active_clients:
            self.active_clients.add(send_address[0])
        if send_address[0] == self.master_osc and len(address_list) == 2:
            self.send(osc.Message('xy', *value_list))
            vx = self.max_run_vx if self.running else self.max_vx
            vy = self.max_run_vy if self.running else self.max_vy
            self.cmd.linear.x = max(min(value_list[0] * vx, vx), -vx)
            if self.holonomic:
                self.cmd.linear.y = max(min(value_list[1] * -vy, vy), -vy)
            else:
                self.cmd.angular.z = max(min(value_list[1] * -vy, vy), -vy)
        elif send_address[0] == self.master_osc and value_list[0] == 0.0:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            if not self.holonomic:
                self.cmd.angular.z = 0.0
            self.zero_xy_command()
        self.pub.publish(self.cmd)

    def w_callback(self, address_list, value_list, send_address):
        if not self.holonomic:
            return
        if send_address[0] not in self.active_clients:
            self.active_clients.add(send_address[0])
        if send_address[0] == self.master_osc and len(address_list) == 2:
            self.send(osc.Message('w', *value_list))
            vw = self.max_run_vw if self.running else self.max_vw
            self.cmd.angular.z = max(min(value_list[0] * -vw, vw), -vw)
        elif send_address[0] == self.master_osc and value_list[0] == 0.0:
            self.cmd.angular.z = 0.0
            self.zero_w_command()
        self.pub.publish(self.cmd)

    def mapping_callback(self, address_list, value_list, send_address):
        if len(address_list) == 2:
            self.holonomic = not self.holonomic
            self.send(osc.Message('w/visible', int(self.holonomic)))
            if self.holonomic:
                self.send(osc.Message('mapping_label', "Holonomic"))
            else:
                self.send(osc.Message('mapping_label', "Differential"))

    def control_callback(self, address_list, value_list, send_address):
        if len(address_list) == 2:
            if send_address[0] not in self.active_clients:
                self.active_clients.add(send_address[0])
                if not self.master_osc:
                    name = self.master_osc
                    self.send(osc.Message('control', 1.0),
                              clients=[self.master_osc])
                    self.send(osc.Message('master', name))
            elif not self.master_osc and value_list[0] == 1.0:
                self.master_osc = send_address[0]
                name = self.master_osc
                self.send(osc.Message('control', 1.0), clients=[self.master_osc])
                self.send(osc.Message('master', name))
            elif value_list[0] == 0.0:
                self.send(osc.Message('control', 0.0))
                self.send(osc.Message('master', ''))
                self.master_osc = None

    def turbo_callback(self, address_list, value_list, send_address):
        if send_address[0] not in self.active_clients:
                self.activeClients[send_address[0]] = address_list[0]
        if send_address[0] == self.master_osc and len(address_list) == 2:
            if value_list[0] == 1.0:
                message = osc.Message("turbo", value_list[0])
                self.send(message)
                self.running = True
            elif value_list[0] == 0.0:
                message = osc.Message("turbo", value_list[0])
                self.send(message)
                self.running = False

    def cb_tabpage_closed(self, client, tabpage):
        if client[0] == self.master_osc:
            self.send(osc.Bundle([osc.Message('control', 0.0),
                                  osc.Message('master', '')]))
            self.master_osc = None
        self.active_clients.discard(client)

    def cb_client_disconnected(self, client):
        if client[0] == self.master_osc:
            self.send(osc.Bundle([osc.Message('control', 0.0),
                                  osc.Message('master', '')]))
            self.master_osc = None
        self.active_clients.discard(client)


