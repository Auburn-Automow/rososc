import roslib; roslib.load_manifest('osc_bridge')
import rospy

import osc_msgs.msg
import osc_msgs.encoding
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from txosc import osc
from txosc import dispatch
from txosc import async

from pytouchosc.layout import Layout
from pytouchosc.tabpage import Tabpage
from oscnode import OSCNode

class TouchOSCNode(OSCNode):
    def __init__(self, layout, name, port, regtype='_osc._udp'):
        super(TouchOSCNode, self).__init__(name, port, regtype)
        self._osc_receiver.addCallback("/accxyz", self.accel_cb)
        self.accel_pub = rospy.Publisher(name+'/accel', Imu)
        
        self.ros_sub = rospy.Subscriber("chatter", String, self.cb)
        
        self.toggle = 0
    
    def accel_cb(self, message, address):
        msg = Imu()
        accels = message.getValues()
        msg.linear_acceleration.x = accels[0]
        msg.linear_acceleration.y = accels[1]
        msg.linear_acceleration.z = accels[2]
        msg.header.frame_id = address[0]
        msg.header.stamp = rospy.Time.now()
        cov = 0.01
        msg.linear_acceleration_covariance = [cov,0,0,0,cov,0,0,0,cov]
        self.accel_pub.publish(msg)
        
    def cb(self, msg):
        element = osc.Message("/1/toggle1/color", 'red')
        if self.toggle == 0:
            self.toggle = 1
        else:
            self.toggle = 0
        self.sendToClients(element)

class AbstractTabpage(object):
    def __init__(self, tabpage, nodeName):
        self.pub = rospy.Publisher(nodeName + '/' + tabpage.name + '/pub', OSC)
        self.sub = rospy.Subscriber(nodeName + '/' + tabpage.name + '/sub', OSC, self.ros_cb)

    def ros_cb(self, msg):
        data = osc_msgs.encoding.decode_osc(msg)
        