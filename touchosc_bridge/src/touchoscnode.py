import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import String
import touchosc_msgs.msg

from txosc import osc
from txosc import dispatch
from txosc import async

from pytouchosc.layout import Layout
from pytouchosc.tabpage import Tabpage
from oscnode import OSCNode

class TouchOSCNode(OSCNode):
    def __init__(self, layout, name, port, regtype='_osc._udp'):
        super(TouchOSCNode, self).__init__(name, port, regtype)
        
        try:
            self.layout = Layout.createFromExisting(layout)
        except IOError:
            rospy.logerr("Layout file not found")
            sys.exit(1)
        self.tabpages = self.layout.getTabpageNames()
        
        # Handle the accelerometer data from the iPad
        self._osc_receiver.addCallback("/accxyz", self.accel_cb)
        self.accel_pub = rospy.Publisher(name+'/accel', Imu)

        self.osc_nodes = {}
        
        for tabpage in self.tabpages:
            self.osc_nodes[tabpage] = dispatch.AddressNode(tabpage)
            self._osc_receiver.addNode(tabpage, self.osc_nodes[tabpage])
        
        self._osc_receiver.addCallback("/1/push3", self.scalable_control_cb)
        self._osc_receiver.addCallback("/1/push3/z", self.scalable_control_cb)
        self.test_pub = rospy.Publisher('asdf', touchosc_msgs.msg.ScalableControl)
        
        self.value = 0.0
        self.z = False
    
    def scalable_control_cb(self, message, address):
        msg = touchosc_msgs.msg.ScalableControl()
        msg.header.stamp = rospy.Time.now()
        addressParts = osc.getAddressParts(message.address)
        msg.common.tabpage = addressParts[0]
        msg.common.name = addressParts[1]
        
        if len(addressParts) == 2:
            self.value = message.getValues()[0]
        else:
            self.z = message.getValues()[0]
        
        msg.value = self.value
        msg.z = bool(self.z)
        self.test_pub.publish(msg)
    
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


class TabpageHandler(object):
    def __init__(self, tabpage, nodeName, oscReceiver, oscSender):
        self.pub = rospy.Publisher(nodeName + '/' + tabpage.name + '/pub', OSC)
        self.sub = rospy.Subscriber(nodeName + '/' + tabpage.name + '/sub', OSC, self.ros_cb)
        self.oscReceiver = oscReceiver
        self.oscSender = oscSender
        
        self.oscReceiver.addCallback('/' + tabpage.name, self.osc_cb)

    def ros_cb(self, msg):
        data = osc_msgs.encoding.decode_osc(msg)
        
    def osc_cb(self, message, address):
        messageDict[message.address] = message.getValues()
        msg = osc_msgs.encoding.encode_osc(messageDict, client)
        self.pub.publish(msg)