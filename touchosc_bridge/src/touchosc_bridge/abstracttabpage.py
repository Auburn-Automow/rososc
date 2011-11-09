"""
Module for base class of user-created tabpage handlers
"""

__package__ = 'touchosc_bridge'
__author__ = 'Michael Carroll <carroll.michael@gmail.com'

import roslib; roslib.load_manifest('touchosc_bridge')
import rospy

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import copy

class AbstractTabpageHandler(object):
    """
    Base class for all TabpageHandlers.  In order to start creating your own 
    Tabpage and handler, inherit from this class.
    """
    def __init__(self, touchosc_interface, handler_name, tabpage_names):
        """
        Initialize a TabpageHandler object.
        
        @type tabpage_names: C{list}
        @param tabpage_names:  List of tabpages that this handler will handle.
        @type handler_name: C{str}
        @param handler_name: Name to be registered with the TouchOscInterface
        """
        self.ros_name = rospy.get_name()
        self.handler_name = handler_name

        self.parent = touchosc_interface

        if type(tabpage_names) is str:
            self.tabpage_names = [tabpage_names]
        elif type(tabpage_names) is list:
            self.tabpage_names = tabpage_names
        else:
            raise ValueError("type(tabpage_names) is not str or list")

        self.osc_node = {}
        for name in self.tabpage_names:
            self.osc_node[name] = {}
            self.osc_node[name][None] = dispatch.AddressNode(name)

        self.ros_publishers = {}
        self.ros_subscribers = {}

    @property
    def osc_nodes(self):
        """
        A dict of all OSC address nodes associated with this handler.
        @type: C{dict}
        """
        returnDict = {}
        for name in self.tabpage_names:
            returnDict[name] = self.osc_node[name][None]
        return returnDict

    def cb_diagnostics_update(self):
        """
        Callback periodically called to update the diagnostics status of the
        tabpage handler.
        
        @return: A status message for the tabpage handler
        @rtype: L{diagnostic_msgs/DiagnosticStatus}
        """
        tabpage_status = DiagnosticStatus()
        tabpage_status.level = tabpage_status.OK
        tabpage_status.name = " ".join([self.parent.ros_name,
                                        self.handler_name,
                                        "Handler"])
        tabpage_status.hardware_id = self.parent.ros_name
        tabpage_status.message = "OK"
        tabpage_status.values = []
        return tabpage_status

    def initialize_tabpage(self):
        """
        Called immedeately after tabpage is loaded.  
        May be used to set default values of controls.
        """
        pass

    def cb_client_connected(self, client):
        """
        Callback when a new client is detected by Bonjour.
        
        @param client: IP address of the client that connected.
        @type client: C{str}
        """
        pass

    def cb_client_disconnected(self, client):
        """
        Callback when a client disconnects, as detected by Bonjour.
        
        @param client: IP address of the clien that disconnected.
        @type client: C{str}
        """
        pass

    def cb_tabpage_active(self, client, tabpage):
        """
        Callback when a client switches to this tabpage.
        
        @param client: IP address of the client that activated the tabpage.
        @type client: C{str}
        @param tabpage: Name of the tabpage opened.
        @type tabpage: C{str}
        """
        pass

    def cb_tabpage_closed(self, client, tabpage):
        """
        Callback when a client switches away from this tabpage
        
        @param client: IP address of the client that closed the tabpage.
        @type client: C{str}
        @param tabpage: Name of the tabpage closed.
        @type tabpage: C{str}
        """
        pass

    def add_osc_callback(self, name, control_callback,
                         tabpages=None, z_callback=None):
        """
        Convenience function for adding OSC callbacks.  Users are welcome to use
        the txosc API for adding their own callbacks, but this tends to make
        it simpler for TouchOSC controls.
        
        control_callback and z_callback must have the function signature:
        
        C{callback(address_list, value_list, send_address)}
        
        @param name: control name (as addressed)
        @type name: C{string}
        @param control_callback: callback function to be called upon match.
        @type control_callback: Function 
        @keyword tabpages: A tabpage name or list of tabpage names (namespaces)
        to add this control to.  If not passed, then the callback will default
        to all tabpage names that are associated with this handler.
        @type tabpages: C{str} or C{list}
        @keyword z_callback: callback function to be called upon control name
        z-state change.  In order for this to work, "Send Z messages" must be
        enabled on the TouchOSC app.
        @type z_callback: function
        """
        if type(tabpages) is list:
            iter_tabpages = tabpages
        elif type(tabpages) is str:
            iter_tabpages = [tabpages]
        else:
            iter_tabpages = self.tabpage_names
        for tabpage in iter_tabpages:
            if tabpage not in self.tabpage_names:
                rospy.logwarn("Tried to add control %s to tabpage %s" %
                               (name, tabpage))
                rospy.logwarn("Cannot add callbacks to an unalias tabage")
                continue
            node = self.osc_node[tabpage]
            node[name] = dispatch.AddressNode(name)
            # Match /tabpage/control value
            node[name].addCallback("*", control_callback)
            if z_callback is not None:
                # Match /tabpage/control/z value
                node[name].addCallback("/z", z_callback)
                # Match /tabpage/control/2/z value
                node[name].addCallback("/[0-9]+/z", z_callback)
                # Match /tabpage/control/2 value
                node[name].addCallback("/[0-9]+", control_callback)
                # Match /tabpage/control/2/2 value
                node[name].addCallback("/[0-9]+/[0-9]+", control_callback)
            else:
                # Match /tabpage/control/z value
                # Match /tabpage/control/2 value
                node[name].addCallback("/*", control_callback)
                # Match /tabpage/control/2/2 value
                node[name].addCallback("/*/*", control_callback)
            node[None].addNode(name, node[name])
