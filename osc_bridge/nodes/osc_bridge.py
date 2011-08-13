#!/usr/bin/env python

import roslib; roslib.load_manifest('osc_bridge')
import rospy

import bonjour


if __name__=="__main__":
    rospy.init_node("osc_bridge")
    rospy.loginfo("ROS OSC Python Node")

    name = rospy.get_param('~name','ROS OSC Node')
    port = int(rospy.get_param('~port','9000'))
    regtype = rospy.get_param('~regtype','_osc._udp')
    
    b = bonjour.bonjour(name,port,regtype)
    b.info = rospy.loginfo
    b.error = rospy.logerr
    b.debug = rospy.logdebug

    b.run()

    while True:
        if rospy.is_shutdown():
            break
        rospy.spin()

    b.shutdown()
    del b
