import roslib; roslib.load_manifest('diagnostics_handler')
import rospy

from txosc import osc
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from touchosc_bridge.abstracttabpage import AbstractTabpageHandler

class DiagnosticsClient(object):
    def __init__(self, client_name, client_type, client_topic):
        self.name = client_name
        self.type = client_type
        self.topic = client_topic

        self.diag_db = None

        self.active = False
        self.paused = False
        self.diagnostics_offset = 0
        self.keyvalue_offset = 0

    def set_diagnostics_offset(self, offset):
        self.diagnostics_offset = offset

    def set_keyvalue_offset(self, offset):
        self.keyvalue_offset = offset

    def update_diagnostics_display(self, display_list, system_status=None,
                                   fader=False):
        it = 1
        to_display = []
        length = len(display_list)
        for (name, color) in display_list[self.diagnostics_offset:]:
            to_display.append(osc.Message("dled%i/color" % it, color))
            to_display.append(osc.Message("dled%i" % it, 1.0))
            to_display.append(osc.Message("dlabel%i" % it, name))
            it += 1
            if it > self.diag_list_length: break;
        while it <= self.diag_list_length:
            to_display.append(osc.Message("dled%i/color" % it, 'gray'))
            to_display.append(osc.Message("dled%i" % it, 0.0))
            to_display.append(osc.Message("dlabel%i" % it, ''))
            it += 1
        if system_status:
            to_display.append(osc.Message("statusled/color", system_status))
            to_display.append(osc.Message("statusled", 1.0))
        if fader:
            if length <= self.diag_list_length:
                value = 1.0
            else:
                value = self.diagnostics_offset / float(length - self.diag_list_length)
            to_display.append(osc.Message("dfader", value))
        return to_display

    def update_keyvalue_display(self, keyvalue_list):
        pass


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
            self.diag_list_length = 16
            self.detail_list_length = 8
            self.client_type = "ipad"
        elif type.lower() == "ipod":
            self.diag_list_length = 10
            self.detail_list_length = 6
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
        if topic.lower() == "/diagnostics":
            self.client_topic = "/diagnostics"
        elif topic.lower() == "/diagnostics_agg":
            self.client_topic = "/diagnostics_agg"
        else:
            raise ValueError("Topic %s is not supported" % topic)

class DiagnosticsMessage(object):
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
        for value in msg.values:
            self.kv_dict[value.key] = value.value
        self.kv_display = sorted(self.kv_dict.keys())

    def get_color(self):
        return self.COLORS[self.status]

    def get_length(self):
        return len(self.kv_dict)

    def get_display_list(self):
        disp_list = []
        for key, value in self.kv_dict.iteritems():
            disp_list.append((key, value))
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
                if not self.diagnostics_data.has_key(message.name):
                    self.diagnostics_data[message.name] = DiagnosticsMessage(message.name)
                self.diagnostics_data[message.name].update(msg.header.stamp, message)
                if message.level != 3:
                    self.system_status = max(self.system_status, message.level)
            self.diagnostics_display = sorted(self.diagnostics_data.keys())

    def get_length(self):
        return len(self.diagnostics_data)

    def get_system_status(self):
        return self.COLORS[self.system_status]

    def get_display_list(self):
        messages = [self.diagnostics_data[x] for x in self.diagnostics_display]
        return [(msg.name, msg.get_color()) for msg in messages]

class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    def __init__(self, touchosc_interface, handler_name, tabpage_names):
        super(DiagnosticsTabpageHandler, self).__init__(touchosc_interface,
                                                        handler_name,
                                                        tabpage_names)

        self.start_topic = rospy.get_param("~" + self.handler_name +
                                           "/start_topic",
                                           "/diagnostics")

        self.add_osc_callback('diagsw', self.stub)

        self.add_osc_callback('darray', self.stub)
        self.add_osc_callback('ddown', self.d_updown_cb)
        self.add_osc_callback('dup', self.d_updown_cb)
        self.add_osc_callback('dfader', self.d_updown_cb)

        self.add_osc_callback('kvup', self.stub)
        self.add_osc_callback('kvdown', self.stub)
        self.add_osc_callback('kvfader', self.stub)

        self.add_osc_callback('pause', self.stub)

        self.osc_clients = {}

        self.diagnostics_data = DiagnosticsData("/diagnostics")
        self.diagnostics_agg_data = DiagnosticsData("/diagnostics_agg")

    def initialize_tabpage(self):
        self.diagnostics_sub = rospy.Subscriber("/diagnostics",
                                                DiagnosticArray,
                                                self.diag_cb)

        self.diagnostics_agg_sub = rospy.Subscriber("/diagnostics_agg",
                                                    DiagnosticArray,
                                                    self.diag_cb)

    def diag_cb(self, msg):
        try:
            topic = msg._connection_header['topic']
        except KeyError:
            return
        if topic == "/diagnostics":
            self.diagnostics_data.add_message(msg)
            display_list = self.diagnostics_data.get_display_list()
            for addr, client in self.osc_clients.iteritems():
                if client.topic == topic:
                    to_display = client.update_diagnostics_display(display_list,
                                                      self.diagnostics_data.get_system_status(),
                                                      fader=True)
                    self.send(osc.Bundle(to_display), clients=[addr])
        elif topic == "/diagnostics_agg":
            self.diagnostics_data.add_message(msg)
        else:
            raise ValueError("Unknown topic type %s" % topic)

    def d_updown_cb(self, address_list, value_list, send_address):
        pass

    def cb_diagnostics_update(self):
        tabpage_status = DiagnosticStatus()
        tabpage_status.level = tabpage_status.OK
        tabpage_status.name = self.handler_name + " Handler"
        tabpage_status.hardware_id = self.ros_name
        tabpage_status.message = "OK"
        tabpage_status.values = []
        for client_name, client in self.osc_clients.iteritems():
            if client.active:
                tabpage_status.values.append(KeyValue(client_name,
                                                      client.topic))
            else:
                tabpage_status.values.append(KeyValue(client_name,
                                                      ""))
            tabpage_status.values.append(KeyValue(client_name, client.type))
        return tabpage_status

    def cb_tabpage_active(self, client, tabpage):
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = True
        else:
            self.cb_client_connected(client)
        pass

    def cb_tabpage_closed(self, client, tabpage):
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = False
        else:
            self.cb_client_connected(client)
            self.osc_clients[client].active = False
        pass

    def cb_client_connected(self, client):
        parent_client = self.parent.clients[client]
        self.osc_clients[client] = DiagnosticsClient(parent_client.servicename,
                                                     parent_client.client_type,
                                                     "/diagnostics")
        self.osc_clients[client].active = True

    def cb_client_disconnected(self, client):
        if self.osc_clients.has_key(client):
            del self.osc_clients[client]

    def stub(self, address_list, value_list, send_address):
        pass

