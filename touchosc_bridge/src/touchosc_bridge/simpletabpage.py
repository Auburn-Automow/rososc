import roslib
roslib.load_manifest('touchosc_bridge')

import rospy

from touchosc_bridge.abstracttabpage import AbstractTabpageHandler

class SimpleTabpageHandler(AbstractTabpageHandler):
    def __init__(self, touchosc_interface, handler_name, tabpage_names):
        super(SimpleTabpageHandler, self).__init__(touchosc_interface,
                                                   handler_name,
                                                   tabpage_names)

        self.add_osc_callback('multitoggle', self.cb_osc_multitoggle,
                                             self.cb_osc_multitoggle_z)

        self.add_osc_callback('xy', self.cb_osc_xy)

    def cb_tabpage_active(self, client, tabpage):
        rospy.loginfo("Client %s is now on tabpage %s" % (client, tabpage))

    def cb_tabpage_closed(self, client, tabpage):
        rospy.loginfo("Client %s closed tabpage %s" % (client, tabpage))

    def cb_client_connected(self, client):
        rospy.loginfo("Client %s connected" % client)

    def cb_client_disconnected(self, client):
        rospy.loginfo("Client %s connected" % client)

    def cb_osc_multitoggle(self, address_list, value_list, send_address):
        rospy.loginfo("Multitoggle Control From: %s" % send_address[0])
        rospy.loginfo(address_list)
        rospy.loginfo(value_list)

    def cb_osc_multitoggle_z(self, address_list, value_list, send_address):
        rospy.loginfo("Multitoggle_z Control From: %s" % send_address[0])
        rospy.loginfo(address_list)
        rospy.loginfo(value_list)

    def cb_osc_xy(self, address_list, value_list, send_address):
        rospy.loginfo("XY Control From: %s" % send_address[0])
        rospy.loginfo(address_list)
        rospy.loginfo(value_list)
