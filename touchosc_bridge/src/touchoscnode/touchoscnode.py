import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

from sensor_msgs.msg import Imu

from txosc import osc
from txosc import dispatch
from txosc import async

from oscnode import OSCNode

class TouchOSCNode(OSCNode):
    def __init__(self, name, port, regtype='_osc._udp'):
        super(TouchOSCNode, self).__init__(name, port, regtype)
        
        # Handle the accelerometer data from the iPad
        self._osc_receiver.addCallback("/accxyz", self.accel_cb)
        self.accel_pub = rospy.Publisher(name + '/accel', Imu)

        self.tabpageHandlers = {}
    
    def accel_cb(self, message, address):
        msg = Imu()
        accels = message.getValues()
        msg.linear_acceleration.x = accels[0]
        msg.linear_acceleration.y = accels[1]
        msg.linear_acceleration.z = accels[2]
        msg.header.frame_id = address[0]
        msg.header.stamp = rospy.Time.now()
        cov = 0.01
        msg.linear_acceleration_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
        self.accel_pub.publish(msg)
        
    def addTabpageHandler(self, tabpageHandler):
        name = tabpageHandler.getTabpageName()
        rospy.loginfo("Adding Tabpage: %s"%name)
        self.tabpageHandlers[name] = tabpageHandler
        self.tabpageHandlers[name].setSender(self.sendToAll)
        self._osc_receiver.addNode(name, self.tabpageHandlers[name].getOscNode())
        
        
        
        
        