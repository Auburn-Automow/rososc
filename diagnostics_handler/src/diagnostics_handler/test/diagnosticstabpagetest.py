import unittest

import roslib; roslib.load_manifest('diagnostics_handler')
import rospy
import time

from diagnostics_handler.diagnosticstabpage import DiagnosticsClient, DiagnosticsMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class Test_DiagnosticsClient(unittest.TestCase):
    def setUp(self):
        self.dc = DiagnosticsClient("Michaels iPhone", "ipod", "/diagnostics")

    def test_constructor(self):
        self.assertIsInstance(self.dc, DiagnosticsClient)

    def test_clientType(self):
        self.assertEqual("ipod", self.dc.type)

    def test_setClientType(self):
        self.dc.client_type = "ipad"
        self.assertEqual("ipad", self.dc.type)

    def test_clientType_unknownType(self):
        with self.assertRaises(ValueError) as cm:
            self.dc.type = "android"
        self.assertEqual(cm.exception.message, "Client type android is not supported")

    def test_clientTopic(self):
        self.assertEqual("/diagnostics", self.dc.topic)

    def test_setClientTopic(self):
        self.dc.client_topic = "/diagnostics_agg"
        self.assertEqual("/diagnostics_agg", self.dc.topic)

    def test_setClientTopic_unknownTopic(self):
        with self.assertRaises(ValueError) as cm:
            self.dc.topic = "notatopic"
        self.assertEqual(cm.exception.message, "Topic notatopic is not supported")

class Test_DiagnosticsMessage(unittest.TestCase):
    def setUp(self):
        self.status = DiagnosticStatus()
        self.status.level = self.status.OK
        self.status.name = "Test Name"
        self.status.hardware_id = "Python Unittest"
        self.status.message = "OK"
        self.status.values = []
        self.status.values.append(KeyValue("Key 1", "Value 1"))
        self.status.values.append(KeyValue("Key 2", "Value 2"))

        self.stamp = rospy.Time.from_sec(time.time())

    def test_constructor(self):
        msg = DiagnosticsMessage(self.status.name)
        self.assertEqual(msg.name, self.status.name)

    def test_update(self):
        msg = DiagnosticsMessage(self.status.name)
        msg.update(self.stamp, self.status)
        self.assertEqual(msg.stamp, self.stamp)
        self.assertEqual(msg.status, self.status.level)

    def test_length(self):
        msg = DiagnosticsMessage(self.status.name)
        msg.update(self.stamp, self.status)
        self.assertEqual(msg.get_length(), 2)

    def test_color(self):
        msg = DiagnosticsMessage(self.status.name)
        msg.update(self.stamp, self.status)
        self.assertEqual(msg.get_color(), "green")

    def test_display_list(self):
        msg = DiagnosticsMessage(self.status.name)
        msg.update(self.stamp, self.status)
        disp = msg.get_display_list()
        self.assertEqual([('Key 1', 'Value 1'), ('Key 2', 'Value 2')],
                         disp)

if __name__ == "__main__":
    unittest.main()
