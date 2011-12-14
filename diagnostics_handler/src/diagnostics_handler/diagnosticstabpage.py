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
        self.detail_offset = 0
        self.detail_display = None

    def set_diagnostics_offset(self, offset):
        self.diagnostics_offset = offset

    def set_detailed_display(self, index):
        detailed_index = self.diagnostics_offset + index
        try:
            (name, _) = self.display_list[detailed_index]
        except:
            return
        try:
            self.detail_display = self.diag_db.diagnostics_data[name]
        except:
            pass

    def set_keyvalue_offset(self, offset):
        self.detail_offset = offset

    def clear_display(self):
        self.display_list = []
        display_list = []
        display_list.extend(self.update_diagnostics_display(display_list, 
                                                            None, True))
        self.detail_display = None
        display_list.extend(self.update_keyvalue_display(True))
        return display_list

    def update_diagnostics_display(self, display_list, system_status=None,
                                   fader=False):
        self.display_list = display_list
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

    def update_keyvalue_display(self, fader=False):
        """
        Updates the detailed key/value display on the iOS device.
        @return: A list of messages to be sent
        @rtype: C{list}
        """
        if self.detail_display is None:
            display_list = []
        else:
            display_list = self.detail_display.get_display_list()
        it = 1
        to_display = []
        length = len(display_list)
        for (key, value) in display_list[self.detail_offset:]:
            to_display.append(osc.Message("key%i"%it, key))
            to_display.append(osc.Message("value%i"%it, value))
            it += 1
            if it > self.detail_list_length: break;
        while it <= self.detail_list_length:
            to_display.append(osc.Message("key%i"%it, ''))
            to_display.append(osc.Message("value%i"%it, ''))
            it +=1
        if fader:
            if length <= self.detail_list_length:
                value = 1.0
            else:
                value = self.detail_offset/float(length - self.detail_list_length)
            to_display.append(osc.Message("kvfader", value))
        
        if self.detail_display is None:
            to_display.append(osc.Message("deviceled/color", "gray"))
            to_display.append(osc.Message("deviceled", 0.0))
            to_display.append(osc.Message("name", ""))
            to_display.append(osc.Message("hardware_id", ""))
            to_display.append(osc.Message("message", ""))
            to_display.append(osc.Message("stamp", ""))
        else:
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
        self.kv_display = []
        for value in msg.values:
            self.kv_dict[value.key] = value.value
            self.kv_display.append(value.key)

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

        self.add_osc_callback('darray', self.darray_cb)
        self.add_osc_callback('ddown', self.d_updown_cb, z_callback=self.stub)
        self.add_osc_callback('dup', self.d_updown_cb, z_callback=self.stub)
        self.add_osc_callback('dfader', self.d_updown_cb, z_callback=self.stub)

        self.add_osc_callback('kvup', self.kv_updown_cb, z_callback=self.stub)
        self.add_osc_callback('kvdown', self.kv_updown_cb, z_callback=self.stub)
        self.add_osc_callback('kvfader', self.kv_updown_cb, z_callback=self.stub)

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
            for addr, client in self.osc_clients.iteritems():
                if client.topic == topic and client.active:
                    to_display = client.update_diagnostics_display(client.diag_db.get_display_list(),
                                                      self.diagnostics_data.get_system_status(),
                                                      fader=True)
                    to_display.extend(client.update_keyvalue_display(fader=True))
                    self.send(osc.Bundle(to_display), clients=[addr])
        elif topic == "/diagnostics_agg":
            self.diagnostics_data.add_message(msg)
        else:
            raise ValueError("Unknown topic type %s" % topic)

    def d_updown_cb(self, address_list, value_list, send_address):
        try:
            client = self.osc_clients[send_address[0]]
        except:
            return
        offset = client.diagnostics_offset
        
        max_offset = client.diag_db.get_length() - client.diag_list_length
        max_offset = 0 if max_offset < 0 else max_offset
        min_offset = 0
        
        if address_list[1] == 'ddown':
            if offset < max_offset:
                client.set_diagnostics_offset(offset+1)
            fader = True
        elif address_list[1] == 'dup':
            if offset > min_offset:
                client.set_diagnostics_offset(offset-1)
            fader = True
        elif address_list[1] == 'dfader':
            client.set_diagnostics_offset(int(round(value_list[0]*max_offset)))
            fader = False
        
        display_list = client.diag_db.get_display_list()  
        to_display = client.update_diagnostics_display(display_list,
                                                       client.diag_db.get_system_status(),
                                                       fader=fader)
        self.send(osc.Bundle(to_display), clients = [send_address[0]])    

    def kv_updown_cb(self, address_list, value_list, send_address):
        try:
            client = self.osc_clients[send_address[0]]
        except:
            return
        offset = client.detail_offset
        
        max_offset = client.detail_display.get_length() - client.detail_list_length
        max_offset = 0 if max_offset < 0 else max_offset
        min_offset = 0
        
        if address_list[1] == 'kvdown':
            if offset < max_offset:
                client.set_keyvalue_offset(offset+1)
            fader = True
        elif address_list[1] == 'kvup':
            if offset > min_offset:
                client.set_keyvalue_offset(offset-1)
            fader = True
        elif address_list[1] == 'kvfader':
            client.set_keyvalue_offset(int(round(value_list[0]*max_offset)))
            fader = False
            
        to_display = client.update_keyvalue_display(fader)
        self.send(osc.Bundle(to_display), clients = [send_address[0]])
    
    def darray_cb(self, address_list, value_list, send_address):
        try:
            client = self.osc_clients[send_address[0]]
        except:
            return
        if len(address_list) == 4:
            client.set_detailed_display(int(address_list[3])-1)
            client.set_keyvalue_offset(0)
            to_display = client.update_keyvalue_display(fader=True)
            self.send(osc.Bundle(to_display), clients = [send_address[0]])
        
    def cb_diagnostics_update(self):
        tabpage_status = DiagnosticStatus()
        tabpage_status.level = tabpage_status.OK
        tabpage_status.name = self.handler_name + " Handler"
        tabpage_status.hardware_id = self.ros_name
        tabpage_status.message = "OK"
        tabpage_status.values = []
        tabpage_status.values.append(KeyValue(key="Number of Clients",
                                              value=str(len(self.osc_clients))))
        for client_name, client in self.osc_clients.iteritems():
            if client.active:
                tabpage_status.values.append(KeyValue(key=client_name,
                                                      value=str(client.topic)))
            tabpage_status.values.append(KeyValue(client_name, str(client.type)))
        return tabpage_status

    def cb_tabpage_active(self, client, tabpage):
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = True
        else:
            self.cb_client_connected(client)
        self.send(osc.Message("diaglbl",self.osc_clients[client].topic),
                  clients = [client])

    def cb_tabpage_closed(self, client, tabpage):
        if self.osc_clients.has_key(client):
            self.osc_clients[client].active = False
        else:
            self.cb_client_connected(client)
            self.osc_clients[client].active = False

    def cb_client_connected(self, client):
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
        self.send(osc.Bundle(to_display),clients = [client])
        self.osc_clients[client].active = True

    def cb_client_disconnected(self, client):
        if self.osc_clients.has_key(client):
            del self.osc_clients[client]

    def stub(self, address_list, value_list, send_address):
        pass

