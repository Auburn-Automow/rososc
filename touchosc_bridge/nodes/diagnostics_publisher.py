#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

import diagnostic_msgs.msg


def diagnostic_tester():
    rospy.init_node("diagnostics_tester")
    pub = rospy.Publisher('diagnostics', diagnostic_msgs.msg.DiagnosticArray)
    while not rospy.is_shutdown():
        dmsg1 = diagnostic_msgs.msg.DiagnosticStatus()
        dmsg1.level = dmsg1.OK
        dmsg1.name = "OK Test"
        dmsg1.message = "A test of an okay diagnostic"
        dmsg1.hardware_id = "1"
        dmsg1.values = [diagnostic_msgs.msg.KeyValue("Temperature","105F"),
                        diagnostic_msgs.msg.KeyValue("Humidity","25%"),
                        diagnostic_msgs.msg.KeyValue("Breakfast","Bagels")]
        
        dmsg2 = diagnostic_msgs.msg.DiagnosticStatus()
        dmsg2.level = dmsg2.WARN
        dmsg2.name = "Warn Test"
        dmsg2.message = "A test of a warn diagnostic"
        dmsg2.hardware_id = "2"
        dmsg2.values = [diagnostic_msgs.msg.KeyValue("Temperature","130F"),
                        diagnostic_msgs.msg.KeyValue("Humidity","25%"),
                        diagnostic_msgs.msg.KeyValue("Breakfast","Chicken")]
        
        dmsg3 = diagnostic_msgs.msg.DiagnosticStatus()
        dmsg3.level = dmsg2.ERROR
        dmsg3.name = "Error Test"
        dmsg3.message = "A test of an error diagnostic"
        dmsg3.hardware_id = "3"
        dmsg3.values = [diagnostic_msgs.msg.KeyValue("Temperature","1000F"),
                        diagnostic_msgs.msg.KeyValue("Humidity","110%"),
                        diagnostic_msgs.msg.KeyValue("Breakfast","Pizza")]

        msg = diagnostic_msgs.msg.DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status = [dmsg1, dmsg2, dmsg3]
        
        rospy.loginfo("Publishing")
        pub.publish(msg)
        rospy.sleep(1.0)
        
if __name__ == '__main__':
    try:
        diagnostic_tester()
    except rospy.ROSInterruptException: pass
        
        
        
