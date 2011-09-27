#!/usr/bin/env python

import roslib; roslib.load_manifest('osc_bridge')
import rospy


import osc_msgs.msg

import liblo
import bonjour


class OSCBridge:
    def __init__(self):
        # Parameters
        self.name = rospy.get_param('~name','ROS OSC Node')
        self.port = int(rospy.get_param('~port','9000'))
        self.regtype = rospy.get_param('~regtype','_osc._udp')

        # Subscribers
        self.sub_osc('/osc/sub',osc_msgs.msg.OSC,self.osc_cb)

        # Publishers
        self.pub_osc('/osc/pub',osc_msgs.msg.OSC)

        # Variables
        self.b = bonjour.bonjour(self.name,self.port,self.regtype)
        self.b.info = rospy.loginfo
        self.b.error = rospy.logerr
        self.b.debug = rospy.logdebug
     
    def run(self):
        self.b.run()

    def osc_cb(self,msg):
        bundle = liblo.Bundle()

        msg.

        return

if __name__=="__main__":
    rospy.init_node("osc_bridge")
    rospy.loginfo("ROS OSC Python Node")
    
    bridge=OSCBridge.OSCBridge()

    while True:
        if rospy.is_shutdown():
            break
        rospy.spin()

