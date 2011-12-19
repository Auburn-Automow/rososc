import roslib; roslib.load_manifest('diagnostics_handler')
import rospy

from txosc import osc
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import copy

from touchosc_bridge.abstracttabpage import AbstractTabpageHandler

class DiagnosticsClient(object):
    """
    A class to represent the display state of an iOS client
    """
    def __init__(self, client_name, client_type, client_topic):
        """
        Constructor for DiagnosticsClient.
        
        @param client_name: Name of the client (Bonjour Service Name)
        @type client_name: C{string}
        @param client_type: Type of the client (ipod/ipad)
        @type client_type: C{string}
        @param client_topic: Topic that the client is subscribed to (/diagnostics
        or /diagnostics_agg)
        """
        self.diag_db = None
        self.active = False
        self._diagnostics_offset = 0
        self._detail_offset = 0
        self._len_diagnostics = 0
        self._len_detail = 0
        self.detail_display = None
        self.display_level = 1
        self.expanded_display = None

        self.name = client_name
        self.type = client_type
        self.topic = client_topic

    def set_detailed_display(self, index):
        detailed_index = self._diagnostics_offset + index
        try:
            (name, _, _) = self.display_list[detailed_index]
            if self.detail_display and name == self.detail_display.name:
                if self.expanded_display and name == self.expanded_display.name:
                    parent = self.expanded_display.get_parent_name()
                    if parent == '':
                        self.expanded_display = None
                        self.display_level = 1
                    else:
                        self.expanded_display = self.diag_db.diagnostics_data[parent]
                        self.display_level -= 1
                    self.detail_display = None
                elif self.diag_db.get_number_children(name) > 0:
                    self.expanded_display = self.detail_display
                    self.display_level += 1
                    self.detail_display = None
            else:
                self.detail_display = self.diag_db.diagnostics_data[name]
        except:
            return

    def clear_diagnostics_display(self):
        it = 1
        to_display = []
        while it <= self._len_diagnostics:
            to_display.append(osc.Message("dled%i/color" % it, 'gray'))
            to_display.append(osc.Message("dled%i" % it, 0.0))
            to_display.append(osc.Message("dlabel%i/color" % it, 'gray'))
            to_display.append(osc.Message("dlabel%i" % it, ''))
            it += 1
        to_display.append(osc.Message("dfader", 0.0))
        return to_display

    def clear_detail_display(self):
        it = 1
        to_display = []
        self.detail_display = None
        while it <= self._len_detail:
            to_display.append(osc.Message("key%i" % it, ''))
            to_display.append(osc.Message("value%i" % it, ''))
            it += 1
        to_display.append(osc.Message("kvfader", 0.0))
        to_display.append(osc.Message("deviceled/color", "gray"))
        to_display.append(osc.Message("deviceled", 0.0))
        to_display.append(osc.Message("name", ""))
        to_display.append(osc.Message("hardware_id", ""))
        to_display.append(osc.Message("message", ""))
        to_display.append(osc.Message("stamp", ""))
        return to_display

    def clear_system_status(self):
        to_display = []
        to_display.append(osc.Message("statusled/color", "gray"))
        to_display.append(osc.Message("statusled", 0.0))
        return to_display

    def clear_display(self):
        to_display = []
        to_display.extend(self.clear_system_status())
        to_display.extend(self.clear_detail_display())
        to_display.extend(self.clear_diagnostics_display())
        return to_display

    def update_diagnostics_display(self, fader=False):
        if self.expanded_display:
            self.display_list = self.diag_db.get_display_list(root=self.expanded_display.name,
                                                              display_level=self.display_level)
        else:
            self.display_list = self.diag_db.get_display_list(display_level=self.display_level)
        it = 1
        to_display = []
        length = len(self.display_list)
        for (long_name, name, color) in self.display_list[self._diagnostics_offset:]:
            to_display.append(osc.Message("dled%i/color" % it, color))
            to_display.append(osc.Message("dled%i" % it, 1.0))
            to_display.append(osc.Message("dlabel%i" % it, name))
            if self.detail_display and long_name == self.detail_display.name:
                to_display.append(osc.Message("dlabel%i/color" % it, "blue"))
            elif self.expanded_display and long_name == self.expanded_display.name:
                to_display.append(osc.Message("dlabel%i/color" % it, "orange"))
            else:
                to_display.append(osc.Message("dlabel%i/color" % it, "gray"))
            it += 1
            if it > self._len_diagnostics: break;
        while it <= self._len_diagnostics:
            to_display.append(osc.Message("dled%i/color" % it, 'gray'))
            to_display.append(osc.Message("dled%i" % it, 0.0))
            to_display.append(osc.Message("dlabel%i" % it, ''))
            to_display.append(osc.Message("dlabel%i/color" % it, 'gray'))
            it += 1
        if fader:
            if length <= self._len_diagnostics:
                value = 1.0
            else:
                value = self.diagnostics_offset / float(length -
                                                        self._len_diagnostics)
            to_display.append(osc.Message("dfader", value))
        to_display.append(osc.Message("statusled/color",
                                      self.diag_db.get_system_status()))
        to_display.append(osc.Message("statusled", 1.0))
        to_display.append(osc.Message("diaglbl", self.topic))
        return to_display

    def update_detail_display(self, fader=False):
        """
        Updates the detailed key/value display on the iOS device.
        @return: A list of messages to be sent
        @rtype: C{list}
        """
        if self.detail_display is None:
            display_list = []
            return self.clear_detail_display()
        else:
            display_list = self.detail_display.get_display_list()
        it = 1
        to_display = []
        length = len(display_list)
        for (key, value) in display_list[self._detail_offset:]:
            to_display.append(osc.Message("key%i" % it, key))
            to_display.append(osc.Message("value%i" % it, value))
            it += 1
            if it > self._len_detail: break;
        while it <= self._len_detail:
            to_display.append(osc.Message("key%i" % it, ''))
            to_display.append(osc.Message("value%i" % it, ''))
            it += 1
        if fader:
            if length <= self._len_detail:
                value = 1.0
            else:
                value = self.detail_offset / float(length -
                                                 self._len_detail)
            to_display.append(osc.Message("kvfader", value))
        to_display.append(osc.Message("deviceled/color",
                                      self.detail_display.get_color()))
        to_display.append(osc.Message("deviceled", 1.0))
        to_display.append(osc.Message("name", self.detail_display.name))
        to_display.append(osc.Message("hardware_id",
                                      self.detail_display.msg.hardware_id))
        to_display.append(osc.Message("message",
                                      self.detail_display.msg.message))
        to_display.append(osc.Message("stamp",
                                      str(self.detail_display.stamp.to_sec())))
        return to_display

    @property
    def type(self):
        """
        Type of the client (ipod/ipad)
        @type: C{string}
        """
        return self.client_type

    @type.setter
    def type(self, type):
        if type.lower() == "ipad":
            self._len_diagnostics = 16
            self._len_detail = 8
            self.client_type = "ipad"
        elif type.lower() == "ipod":
            self._len_diagnostics = 10
            self._len_detail = 6
            self.client_type = "ipod"
        else:
            raise ValueError("Client type %s is not supported" % type)

    @property
    def topic(self):
        """
        Topic that the client is subscribed to
        @type: C{string}
        """
        return self.client_topic

    @topic.setter
    def topic(self, topic):
        self.expanded_display = None
        self.display_level = 1
        self.detail_display = None
        self._diagnostics_offset = 0
        if topic.lower() == "/diagnostics":
            self.client_topic = "/diagnostics"
        elif topic.lower() == "/diagnostics_agg":
            self.client_topic = "/diagnostics_agg"
        else:
            raise ValueError("Topic %s is not supported" % topic)

    @property
    def diagnostics_offset(self):
        """
        List offset for the diagnostics display
        @type: C{int}
        """
        return self._diagnostics_offset

    @diagnostics_offset.setter
    def diagnostics_offset(self, offset):
        max_offset = len(self.display_list) - self._len_diagnostics
        max_offset = 0 if max_offset < 0 else max_offset
        min_offset = 0
        if offset <= max_offset and offset >= min_offset:
            self._diagnostics_offset = offset

    @property
    def detail_offset(self):
        """
        List offset for the detailed key/value display
        @type: C{int}
        """
        return self._detail_offset

    @detail_offset.setter
    def detail_offset(self, offset):
        if self.detail_display is not None:
            max_offset = self.detail_display.get_length() - self._len_detail
            max_offset = 0 if max_offset < 0 else max_offset
            min_offset = 0
            if offset <= max_offset and offset >= min_offset:
                self._detail_offset = offset
        else:
            self._detail_offset = offset

    def get_max_detail_offset(self):
        max_offset = self.detail_display.get_length() - self._len_detail
        max_offset = 0 if max_offset < 0 else max_offset
        return max_offset

    def get_max_diagnostics_offset(self):
        max_offset = len(self.display_list) - self._len_diagnostics
        max_offset = 0 if max_offset < 0 else max_offset
        return max_offset

class DiagnosticsItem(object):
    COLORS = {0: "green", 1: "yellow", 2: "red", 3:"gray"}
    def __init__(self, name):
        self.name = name
        self.kv_dict = {}
        self.stamp = None
        self.status = 3
        self.kv_display = []

    def update(self, stamp, msg):
        self.stamp = stamp
        self.status = msg.level
        self.msg = msg
        self.kv_display = []
        for value in msg.values:
            self.kv_dict[value.key] = value.value
            self.kv_display.append(value.key)

    def get_parent_name(self):
        return ('/'.join(self.name.split('/')[:-1])).strip()

    def get_nice_name(self):
        return self.name.split('/')[-1]

    def get_level(self):
        return len(self.name.split('/')) - 1

    def get_color(self):
        return self.COLORS[self.status]

    def get_length(self):
        return len(self.kv_dict)

    def get_display_list(self):
        disp_list = []
        for key in self.kv_display:
            disp_list.append((key, self.kv_dict[key]))
        return disp_list

class DiagnosticsData(object):
    COLORS = {0: "green", 1: "yellow", 2: "red", 3:"gray"}
    def __init__(self, topic):
        self.diagnostics_data = {}
        self.diagnostics_display = []
        self.system_status = DiagnosticStatus.OK
        self.topic = topic

    def add_message(self, msg):
        if msg is not None:
            for message in msg.status:
                if not message.name and not self._has_warned_no_name:
                    rospy.logwarn('''DiagnosticStatus message with no "name". 
                    Unable to add to handler. Message: %s, hardware ID: %s, 
                    level: %d''' % (s.message, s.hardware_id, s.level))
                    self._has_warned_no_name = True
                if not message.name:
                    continue
                if len(message.name) > 0 and message.name[0] != '/':
                    message.name = '/' + message.name
                if not self.diagnostics_data.has_key(message.name):
                    self.diagnostics_data[message.name] = DiagnosticsItem(message.name)
                self.diagnostics_data[message.name].update(msg.header.stamp, message)
                if message.level != 3:
                    self.system_status = max(self.system_status, message.level)
            self.diagnostics_display = sorted(self.diagnostics_data.keys())

    def get_number_children(self, parent):
        children = 0
        for item in self.diagnostics_data.itervalues():
            if item.get_parent_name() == parent:
                children += 1
        return children

    def get_length(self):
        return len(self.diagnostics_data)

    def get_system_status(self):
        return self.COLORS[self.system_status]

    def get_display_list(self, root=None, display_level=None):
        if display_level and root:
            messages = [self.diagnostics_data[x] for x in self.diagnostics_display]
            display_list = []
            rootNode = self.diagnostics_data[root]
            display_list.append((rootNode.name, rootNode.get_nice_name(), rootNode.get_color()))
            for msg in messages:
                if msg.get_parent_name() == root:
                    display_list.append((msg.name, msg.get_nice_name(), msg.get_color()))
            return display_list
        if display_level and not root:
            messages = [self.diagnostics_data[x] for x in self.diagnostics_display]
            display_list = []
            for msg in messages:
                if msg.get_level() == display_level:
                    display_list.append((msg.name, msg.get_nice_name(), msg.get_color()))
            return display_list
        else:
            messages = [self.diagnostics_data[x] for x in self.diagnostics_display]
            return [(msg.name, msg.get_nice_name(), msg.get_color()) for msg in messages]

class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    def __init__(self, touchosc_interface, handler_name, tabpage_names):
        super(DiagnosticsTabpageHandler, self).__init__(touchosc_interface,
                                                        handler_name,
                                                        tabpage_names)
        self.start_topic = rospy.get_param("~" + self.handler_name +
                                           "/start_topic",
                                           "/diagnostics")
        self.add_osc_callback('diagsw', self.diagsw_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('darray', self.darray_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('ddown', self.d_updown_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('dup', self.d_updown_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('dfader', self.d_updown_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('kvup', self.kv_updown_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('kvdown', self.kv_updown_cb,
                              z_callback=self.fallback)
        self.add_osc_callback('kvfader', self.kv_updown_cb,
                              z_callback=self.fallback)
        self.osc_clients = {}
        self.diagnostics_data = DiagnosticsData("/diagnostics")
        self.diagnostics_agg_data = DiagnosticsData("/diagnostics_agg")

    def initialize_tabpage(self):
        self.diagnostics_sub = rospy.Subscriber("diagnostics",
                                                DiagnosticArray,
                                                self.diag_cb)
        self.diagnostics_agg_sub = rospy.Subscriber("diagnostics_agg",
                                                    DiagnosticArray,
                                                    self.diag_agg_cb)

    def diag_agg_cb(self, msg):
        self.diagnostics_agg_data.add_message(msg)
        for addr, client in self.osc_clients.iteritems():
            if client.active:
                to_display = client.update_diagnostics_display(fader=True)
                to_display.extend(client.update_detail_display(fader=True))
                self.send(osc.Bundle(to_display), clients=[addr])
        self.send(osc.Message("rostime", str(rospy.Time.now().to_sec())))

    def diag_cb(self, msg):
        self.diagnostics_data.add_message(msg)
        for addr, client in self.osc_clients.iteritems():
            if client.active:
                to_display = client.update_diagnostics_display(fader=True)
                to_display.extend(client.update_detail_display(fader=True))
                self.send(osc.Bundle(to_display), clients=[addr])
        self.send(osc.Message("rostime", str(rospy.Time.now().to_sec())))

    def diagsw_cb(self, address_list, value_list, send_address):
        if not self.osc_clients.has_key(send_address[0]):
            self.cb_client_connected(send_address[0])
        if value_list[0] == 0.0:
            client = self.osc_clients[send_address[0]]
            to_display = client.clear_display()
            if client.topic == "/diagnostics":
                client.topic = "/diagnostics_agg"
                client.diag_db = self.diagnostics_agg_data
            else:
                client.topic = "/diagnostics"
                client.diag_db = self.diagnostics_data
            to_display.append(osc.Message("diaglbl", client.topic))
            to_display.extend(client.update_diagnostics_display(fader=True))
            to_display.extend(client.update_detail_display(fader=True))
            self.send(osc.Bundle(to_display), clients=[send_address[0]])

    def d_updown_cb(self, address_list, value_list, send_address):
        if self.osc_clients.has_key(send_address[0]):
            client = self.osc_clients[send_address[0]]
            offset = client.diagnostics_offset
            if address_list[1] == 'ddown':
                client.diagnostics_offset += 1
                fader = True
            elif address_list[1] == 'dup':
                client.diagnostics_offset -= 1
                fader = True
            elif address_list[1] == 'dfader':
                max_offset = client.get_max_diagnostics_offset()
                client.diagnostics_offset = int(round(value_list[0] * max_offset))
                fader = False
            to_display = client.update_diagnostics_display(fader=fader)
            self.send(osc.Bundle(to_display), clients=[send_address[0]])

    def kv_updown_cb(self, address_list, value_list, send_address):
        """
        Callback for up and down buttons as well as the slider for the 
        detailed display list
        
        @param address_list: OSC address of the incoming message
        @param value_list: OSC value arguments of the incoming message
        @param send_address: IP and port of the originating client
        """
        if self.osc_clients.has_key(send_address[0]):
            client = self.osc_clients[send_address[0]]
            if address_list[1] == 'kvdown':
                client.detail_offset += 1
                fader = True
            elif address_list[1] == 'kvup':
                client.detail_offset -= 1
                fader = True
            elif address_list[1] == 'kvfader':
                max_offset = client.get_max_detail_offset()
                client.detail_offset = int(round(value_list[0] * max_offset))
                fader = False
            to_display = client.update_detail_display(fader=fader)
            self.send(osc.Bundle(to_display), clients=[send_address[0]])

    def darray_cb(self, address_list, value_list, send_address):
        try:
            client = self.osc_clients[send_address[0]]
        except:
            return
        if len(address_list) == 4:
            client.set_detailed_display(int(address_list[3]) - 1)
            client.detail_offset = 0
            to_display = client.update_diagnostics_display(fader=True)
            to_display.extend(client.update_detail_display(fader=True))
            self.send(osc.Bundle(to_display), clients=[send_address[0]])

    def cb_diagnostics_update(self):
        tabpage_status = DiagnosticStatus()
        tabpage_status.level = tabpage_status.OK
        tabpage_status.name = self.handler_name + " Handler"
        tabpage_status.hardware_id = self.ros_name
        tabpage_status.message = "OK"
        tabpage_status.values = []
        tabpage_status.values.append(KeyValue(key="Number of Clients",
                                              value=str(len(self.osc_clients))))
        return tabpage_status

    def cb_tabpage_active(self, client, tabpage):
        """
        Callback for when a client has opened a tabpage handled by this handler
        """
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = True
        else:
            self.cb_client_connected(client)
        self.send(osc.Message("diaglbl", self.osc_clients[client].topic),
                  clients=[client])

    def cb_tabpage_closed(self, client, tabpage):
        """
        Callback for when the client has closed a tabpage handled by this 
        handler
        """
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = False
        else:
            self.cb_client_connected(client)
            self.osc_clients[client].active = False

    def cb_client_connected(self, client):
        """
        Callback for when a client has connected.
        
        Add the client to the local dictionary of clients and initialize it.
        """
        parent_client = self.parent.clients[client]
        self.osc_clients[client] = DiagnosticsClient(parent_client.servicename,
                                                     parent_client.client_type,
                                                     self.start_topic)
        if self.start_topic == "/diagnostics":
            self.osc_clients[client].diag_db = self.diagnostics_data
        elif self.start_topic == "/diagnostics_agg":
            self.osc_clients[client].diag_db = self.diagnostics_agg_data
        else:
            raise ValueError
        to_display = []
        to_display.extend(self.osc_clients[client].clear_display())
        to_display.append(osc.Message("diaglbl", self.osc_clients[client].topic))
        self.send(osc.Bundle(to_display), clients=[client])
        self.osc_clients[client].active = True

    def cb_client_disconnected(self, client):
        """
        Callback for when a client has disconnected.
        
        Remove the client from the local dictionary of clients
        """
        if self.osc_clients.has_key(client):
            del self.osc_clients[client]

