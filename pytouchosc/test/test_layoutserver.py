import unittest
import os
import sys
import time

import httplib

sys.path.append(os.path.abspath('../src'))
from pytouchosc.layout import Layout
from pytouchosc.layoutserver import LayoutServer


layoutPath = os.path.abspath('./layouts')
layoutFile = 'ROS-Demo-iPad.touchosc'
layoutBare = 'index.xml'

class TestLayoutServer(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutFile)
        self.layout = Layout.createFromExisting(self.path)

    def test_constructor(self):
        layoutserver = LayoutServer(self.layout,
                                    "Test Server",
                                    10000)
        self.assertIsInstance(layoutserver, LayoutServer)

    def test_createFromExisting(self):
        layoutserver = LayoutServer.createFromExisting(self.path,
                                                       "Test Server",
                                                       10000)
        self.assertIsInstance(layoutserver, LayoutServer)

class TestLayoutServer_run(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutFile)
        self.layout = Layout.createFromExisting(self.path)
        self.layoutserver = LayoutServer(self.layout,
                                         "Test Server",
                                         10020)
        self.layoutserver.run()
        self.conn = httplib.HTTPConnection("localhost:10020")
        
    def tearDown(self):
        self.layoutserver.stop()

    def test_checkHeaders(self):
        """
        Check that the headers are sent correctly"""
        self.conn.request("GET", "/")
        r1 = self.conn.getresponse()
        self.assertEqual(r1.status, 200)
        self.assertEqual(r1.reason, "OK")
        self.assertEqual(r1.getheader('content-type'), 'application/touchosc',
            "Header: 'content-type' does not match.")
        self.assertEqual(r1.getheader('content-disposition'),
            ('attachment; filename="ROS-Demo-iPad.touchosc"'),
            "Header: 'content-disposition' does not match.")
        self.assertEqual(r1.read(), self.layout.toXml(), "Contents don't match")
        self.conn.close()


class TestLayoutServer_runExisting(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutFile)
        self.layout = Layout.createFromExisting(self.path)
        self.layoutserver = LayoutServer.createFromExisting(self.path,
                                         "Test Server",
                                         10030)
        self.layoutserver.run()
        self.conn = httplib.HTTPConnection("localhost:10030")
        
    def tearDown(self):
        self.layoutserver.stop()

    def test_checkHeaders(self):
        """
        Check that the headers are sent correctly"""
        self.conn.request("GET", "/")
        r1 = self.conn.getresponse()
        self.assertEqual(r1.status, 200)
        self.assertEqual(r1.reason, "OK")
        self.assertEqual(r1.getheader('content-type'), 'application/touchosc',
            "Header: 'content-type' does not match.")
        self.assertEqual(r1.getheader('content-disposition'),
            ('attachment; filename="ROS-Demo-iPad.touchosc"'),
            "Header: 'content-disposition' does not match.")
        self.assertEqual(r1.read(), self.layout.toXml(), "Contents don't match")
        self.conn.close()

def suite():
    load = unittest.TestLoader()
    suite = load.loadTestsFromTestCase(TestLayoutServer)
    suite.addTests(load.loadTestsFromTestCase(TestLayoutServer_run))
    suite.addTests(load.loadTestsFromTestCase(TestLayoutServer_runExisting)) 
    return suite

def rostest():
    suite = []
    suite.append(['TestLayoutServer', TestLayoutServer])
    suite.append(['TestLoading', TestLayoutServer_run])
    suite.append(['TestLoadingFromFile', TestLayoutServer_runExisting])
    return suite

if __name__ == "__main__":
    unittest.main()