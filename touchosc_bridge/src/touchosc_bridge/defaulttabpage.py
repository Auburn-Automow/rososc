__package__ = 'touchosc_bridge'
__author__ = 'Michael Carroll <carroll.michael@gmail.com'

import roslib; roslib.load_manifest('touchosc_bridge')

from txosc import osc
from txosc import dispatch

from abstracttabpage import AbstractTabpageHandler
import pytouchosc
import touchosc_msgs.msg
import rospy

class DefaultTabpageHandler(AbstractTabpageHandler):
    """
    Child class of AbstractTabpageHandler to create ROS publishers and 
    subscribers for each of the controls in a loaded tabpage.
    """
    def __init__(self, touchosc_interface, tabpage):
        """
        Initialize DefaultTabpageHandler
        
        @type touchosc_interface:
        @param touchosc_interface:
        @type handler_name:
        @param handler_name:
        @type tabpage: C{pytouchosc.tabpage}
        @param tabpage: Parsed-XML format of a tabpage to create a default 
        handler for.
        """
        super(DefaultTabpageHandler, self).__init__(touchosc_interface,
                                                    tabpage.name,
                                                    [tabpage.name])
        self.tabpage = tabpage
        self.message_dict = self.tabpage.getMessages()
        ros_prefix = self.ros_name + '/' + self.tabpage_names[0] + '/'

        for control in self.tabpage.iterchildren():
            control_type = type(control)
            ros_cb = None
            osc_cb = None
            osc_z_cb = None
            if control_type is pytouchosc.controls.Button or \
                control_type is pytouchosc.controls.Dial:
                msg_type = touchosc_msgs.msg.ScalableControl
                ros_cb = self.scalable_control_ros_cb
                osc_cb = self.scalable_control_osc_cb
                osc_z_cb = self.scalable_control_z_osc_cb
            elif control_type is pytouchosc.controls.LED:
                msg_type = touchosc_msgs.msg.ScalableControl
                ros_cb = self.scalable_control_ros_cb
            elif control_type is pytouchosc.controls.Time or \
                control_type is pytouchosc.controls.TextField:
                msg_type = touchosc_msgs.msg.TouchOSC_Common
                ros_cb = self.common_ros_cb
            elif control_type is pytouchosc.controls.Label:
                msg_type = touchosc_msgs.msg.Label
                ros_cb = self.label_ros_cb
            elif control_type is pytouchosc.controls.Encoder:
                msg_type = touchosc_msgs.msg.ScalableControl
                ros_cb = self.common_ros_cb
                osc_cb = self.scalable_control_osc_cb
                osc_z_cb = self.scalable_control_z_osc_cb
            elif control_type is pytouchosc.controls.MultiButton:
                msg_type = touchosc_msgs.msg.MultiButton
                ros_cb = self.multibutton_ros_cb
                osc_cb = self.multibutton_osc_cb
                osc_z_cb = self.multibutton_z_osc_cb
            elif control_type is pytouchosc.controls.MultiDial:
                msg_type = touchosc_msgs.msg.MultiFader
                ros_cb = self.multifader_ros_cb
                osc_cb = self.multifader_osc_cb
                osc_z_cb = self.multifader_z_osc_cb
            elif control_type is pytouchosc.controls.XYPad:
                msg_type = touchosc_msgs.msg.XYPad
                ros_cb = self.xypad_ros_cb
                osc_cb = self.xypad_osc_cb
                osc_z_cb = self.xypad_z_osc_cb
            elif control_type is pytouchosc.controls.MultiXYPad:
                msg_type = touchosc_msgs.msg.MultiXYPad
                ros_cb = self.common_ros_cb
                osc_cb = self.multixypad_osc_cb
                osc_z_cb = self.multixypad_z_osc_cb
            if msg_type is not None:
                address = ros_prefix + control.name
                self.ros_subscribers[control.name] = rospy.Subscriber(address,
                                                                      msg_type,
                                                                      ros_cb)
                self.ros_publishers[control.name] = rospy.Publisher(address,
                                                                    msg_type)
                if osc_cb:
                    self.add_osc_callback(control.name, osc_cb,
                                          z_callback=osc_z_cb)

    def osc_populate_common(self, msg):
        try:
            topic = msg._connection_header['topic'].split('/')
            tabpage_name = topic[2]
            control_name = topic[3]
        except KeyError:
            tabpage_name = ""
            control_name = ""

        to_send = []
        control_dict = self.message_dict[control_name]
        if msg.common.color != '':
            to_send.append(osc.Message('/'.join([control_name, 'color']),
                                      msg.common.color))
            control_dict['color'] = msg.common.color
        if (msg.common.x != 0
            and msg.common.x != int(control_dict['position']['x'])):
            to_send.append(osc.Message('/'.join([control_name, 'position/x']),
                                       msg.common.x))
            control_dict['position']['x'] = msg.common.x
        if (msg.common.y != 0
            and msg.common.y != int(control_dict['position']['y'])):
            to_send.append(osc.Message('/'.join([control_name, 'position/y']),
                                      msg.common.y))
            control_dict['position']['y'] = msg.common.y
        if (msg.common.width != 0
            and msg.common.width != int(control_dict['size']['w'])):
            to_send.append(osc.Message('/'.join([control_name, 'size/w']),
                                      msg.common.width))
            control_dict['size']['w'] = msg.common.width
        if (msg.common.height != 0
            and msg.common.height != int(control_dict['size']['h'])):
            to_send.append(osc.Message('/'.join([control_name, 'size/h']),
                                      msg.common.height))
            control_dict['size']['h'] = msg.common.height
        if msg.common.visible != '':
            send = 0 if msg.common.visible.lower() == 'false' else 1
            if bool(send) != bool(control_dict['visibility']):
                to_send.append(osc.Message('/'.join([control_name, 'visible']),
                                          send))
                control_dict['visibility'] = bool(send)
        return (control_name, control_dict, to_send)

    def ros_populate_common(self, control_name):
        common_msg = touchosc_msgs.msg.CommonProperties()
        control_dict = self.message_dict[control_name]
        common_msg.tabpage = self.tabpage_names[0]
        common_msg.name = control_name
        common_msg.color = control_dict['color']
        common_msg.x = int(control_dict['position']['x'])
        common_msg.y = int(control_dict['position']['y'])
        common_msg.width = int(control_dict['size']['w'])
        common_msg.height = int(control_dict['size']['h'])
        common_msg.visible = str(control_dict['visibility'])
        return common_msg

    def send_osc_message(self, frame_id, to_send):
        self.send(osc.Bundle(to_send))

    def common_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (_, _, to_send) = self.osc_populate_common(msg)
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def scalable_control_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (control, control_dict, to_send) = self.osc_populate_common(msg)
                if msg.value != control_dict[None]:
                    control_dict[None] = msg.value
                    to_send.append(osc.Message(control, msg.value))
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def label_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (control, control_dict, to_send) = self.osc_populate_common(msg)
                if msg.value != control_dict['text']:
                    control_dict['text'] = msg.value
                    to_send.append(osc.Message(control, msg.value))
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def multibutton_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (control, control_dict, to_send) = self.osc_populate_common(msg)
                if list(msg.values) != control_dict[None]:
                    control_dict[None] = list(msg.values)
                    to_send.append(osc.Message(control, *control_dict[None]))
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def multifader_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (control, control_dict, to_send) = self.osc_populate_common(msg)
                if list(msg.values) != control_dict[None]:
                    control_dict[None] = list(msg.values)
                    to_send.append(osc.Message(control, *control_dict[None]))
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def xypad_ros_cb(self, msg):
        try:
            if msg._connection_header['callerid'] != self.ros_name:
                (control, control_dict, to_send) = self.osc_populate_common(msg)
                if [msg.x, msg.y] != control_dict[None]:
                    control_dict[None] = [msg.x, msg.y]
                    to_send.append(osc.Message(control, *control_dict[None]))
                self.send_osc_message(msg.header.frame_id, to_send)
        except KeyError:
            pass

    def scalable_control_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict[None] = float(value_list[0])
        msg = touchosc_msgs.msg.ScalableControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.value = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def scalable_control_z_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict['z'] = bool(value_list[0])
        msg = touchosc_msgs.msg.ScalableControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.value = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def multibutton_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        x = int(address_list[2]) - 1
        y = int(address_list[3]) - 1
        control_dict[None][y + x * control_dict['dim_y']] = value_list[0]
        msg = touchosc_msgs.msg.MultiButton()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.dimension = [control_dict['dim_x'], control_dict['dim_y']]
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.values = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def multibutton_z_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict['z'] = bool(value_list[0])
        msg = touchosc_msgs.msg.MultiButton()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.dimension = [control_dict['dim_x'], control_dict['dim_y']]
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.values = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def multifader_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        pos = int(address_list[2]) - 1
        control_dict[None][pos] = value_list[0]
        msg = touchosc_msgs.msg.MultiFader()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.dimension = control_dict['number']
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.values = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def multifader_z_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict['z'] = bool(value_list[0])
        msg = touchosc_msgs.msg.MultiFader()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.dimension = control_dict['number']
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.values = control_dict[None]
        self.ros_publishers[control_name].publish(msg)

    def multixypad_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        touch_number = int(address_list[2])
        control_dict['x'][touch_number - 1] = value_list[0]
        control_dict['y'][touch_number - 1] = value_list[1]
        msg = touchosc_msgs.msg.MultiXYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.x = control_dict['x']
        msg.y = control_dict['y']
        self.ros_publishers[control_name].publish(msg)

    def multixypad_z_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        touch_number = int(address_list[2])
        control_dict['z'][touch_number - 1] = bool(value_list[0])
        if not bool(value_list[0]):
            control_dict['x'][touch_number - 1] = 0.0
            control_dict['y'][touch_number - 1] = 0.0
        msg = touchosc_msgs.msg.MultiXYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.z = control_dict['z']
        msg.x = control_dict['x']
        msg.y = control_dict['y']
        self.ros_publishers[control_name].publish(msg)

    def xypad_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict[None] = value_list
        msg = touchosc_msgs.msg.XYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.x = control_dict[None][0]
        msg.y = control_dict[None][1]
        msg.z = control_dict['z']
        self.ros_publishers[control_name].publish(msg)

    def xypad_z_osc_cb(self, address_list, value_list, send_address):
        control_name = address_list[1]
        control_dict = self.message_dict[control_name]
        control_dict['z'] = value_list[0]
        msg = touchosc_msgs.msg.XYPad()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = send_address[0]
        msg.common = self.ros_populate_common(control_name)
        msg.range = [control_dict['scalef'], control_dict['scalet']]
        msg.x = control_dict[None][0]
        msg.y = control_dict[None][1]
        msg.z = control_dict['z']
        self.ros_publishers[control_name].publish(msg)
