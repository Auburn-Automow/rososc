#!/usr/bin/env python

PKG = 'pytouchosc'
import roslib; roslib.load_manifest(PKG)
import rospy

import sys
import unittest
import httplib
import time

import rostest

from pytouchosc.layout import Layout
from pytouchosc.bonjour import Bonjour

class TestLayoutServer(unittest.TestCase):
    def setUp(self):
        self.name = rospy.get_param('~name', 'OSC Server')
        self.port = int(rospy.get_param('~port', '9658'))
        self.file = rospy.get_param('~layout_file', rospy.get_param('layout_file', None))  
        self.layout = Layout.createFromExisting(self.file)
        self.clients = {}
        
    def test_checkHeaders(self):
        conn = httplib.HTTPConnection("localhost",port=self.port)
        conn.request("GET", "/")

        r1 = conn.getresponse()
        self.assertEqual(r1.status, 200)
        self.assertEqual(r1.reason, "OK")
        self.assertEqual(r1.getheader('content-type'), 'application/touchosc',
            "Header: 'content-type' does not match.")
        self.assertEqual(r1.getheader('content-disposition'),
            ('attachment; filename="ROS-Demo-iPad.touchosc"'),
            "Header: 'content-disposition' does not match.")
        self.assertEqual(r1.read(), self.layout.toXml(), "Contents don't match")
        conn.close()

    def test_bonjour(self):
        try:
            b = Bonjour("Bonjour listener", 1234, '_touchosceditor._tcp')
            b.setClientCallback(self.clientCallback)
            b.run_browser()
            while self.clients == {}:
                time.sleep(1.0)
            self.assertEqual(self.clients.keys()[0], self.name.encode('utf-8'), "Bonjour: Registration Failed")
            vals = self.clients.values()[0]
            self.assertEqual(vals['port'], self.port, "Bonjour: Registered Port Mismatch")
        finally:
            b.stop_browser()

    def clientCallback(self, clientDict):
        self.clients = clientDict

if __name__ == '__main__':
	rospy.init_node('layoutserver_test')
	rostest.rosrun('pytouchosc', 'test_layoutserver_node', TestLayoutServer)