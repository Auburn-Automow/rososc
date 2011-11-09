import roslib
roslib.load_manifest('touchosc_bridge')

import rospy

from touchosc_bridge.abstracttabpage import AbstractTabpageHandler

class SimpleTabpageHandler(AbstractTabpageHandler):
    def __init__(self, touchosc_interface, handler_name, tabpage_names):
        super(SimpleTabpageHandler, self).__init__(touchosc_interface,
                                                   handler_name,
                                                   tabpage_names)

        # Add a callback for a control XY on all aliased tabpages
        self.add_osc_callback('xy', self.cb_osc_xy)

        # Add a callback for a control multitoggle on all aliased tabpages
        # This one also includes a separate z state callback
        self.add_osc_callback('multipush', self.cb_osc_multitoggle,
                              tabpages=['1'],
                              z_callback=self.cb_osc_multitoggle_z)

        # Add a callback for a control multifader on tabpage 2.
        self.add_osc_callback('multifader', self.cb_osc_multifader,
                              tabpages=['1'],
                              z_callback=self.cb_osc_multifader_z)

        self.add_osc_callback('multixy', self.multixy,
                              tabpages=['2'])


    def multixy(self, address_list, value_list, send_address):
        rospy.loginfo("multixy From: %s" % send_address[0])
        print "/".join(address_list)

    def multixy_z(self, address_list, value_list, send_address):
        rospy.loginfo("multixy Z From: %s" % send_address[0])
        print "/".join(address_list)

    def cb_tabpage_active(self, client, tabpage):
        rospy.loginfo("Client %s is now on tabpage %s" % (client, tabpage))

    def cb_tabpage_closed(self, client, tabpage):
        rospy.loginfo("Client %s closed tabpage %s" % (client, tabpage))

    def cb_client_connected(self, client):
        rospy.loginfo("Client %s connected" % client)

    def cb_client_disconnected(self, client):
        rospy.loginfo("Client %s disconnected" % client)

    def cb_osc_multitoggle(self, address_list, value_list, send_address):
        rospy.loginfo("Multitoggle Control From: %s" % send_address[0])
        print "/".join(address_list)

    def cb_osc_multitoggle_z(self, address_list, value_list, send_address):
        rospy.loginfo("Multitoggle Z From: %s" % send_address[0])
        print "/".join(address_list)

    def cb_osc_xy(self, address_list, value_list, send_address):
        rospy.loginfo("XY Control From: %s" % send_address[0])
        print "/".join(address_list)

    def cb_osc_multifader(self, address_list, value_list, send_address):
        rospy.loginfo("Multifader Control From: %s" % send_address[0])
        print "/".join(address_list)

    def cb_osc_multifader_z(self, address_list, value_list, send_address):
        rospy.loginfo("Multifader Z From: %s" % send_address[0])

