#!/usr/bin/env python

import roslib; roslib.load_manifest('touchosc_bridge')

import unittest
import os
import sys


from touchosc_bridge.common import AggregateControl, DisplayList
import pytouchosc.controls as controls
import pytouchosc.layout as layout
from pytouchosc.tabpage import Tabpage

class Test_AggregateControl(unittest.TestCase):
    def setUp(self):
        self.aggregate_control = AggregateControl('control_name')
        self.createControls()

    def createControls(self):
        self.controls = []
        for i in range(5):
            c = controls.control_factory("labelv", "field%i"%i)
            c.x = 0
            c.y = i * 20
            c.width = 100
            c.height = 20
            self.controls.append(c)

    def test_constructor(self):
        self.assertIsInstance(self.aggregate_control, AggregateControl)
        self.assertEqual(self.aggregate_control.x, 0)
        self.assertEqual(self.aggregate_control.y, 0)

    def test_emptyobject(self):
        """
        Before any controls are added, width and height should be zero
        """
        self.assertEqual(self.aggregate_control.width, 0)
        self.assertEqual(self.aggregate_control.height, 0)

    def test_set_x(self):
        self.aggregate_control.x = 80
        self.assertEqual(self.aggregate_control.x, 80)

    def test_set_y(self):
        self.aggregate_control.y = 100
        self.assertEqual(self.aggregate_control.y, 100)

    def test_set_x_noninteger(self):
        self.aggregate_control.x = 50.53
        self.assertEqual(self.aggregate_control.x, 50)

    def test_set_y_noninteger(self):
        self.aggregate_control.y = 500.4
        self.assertEqual(self.aggregate_control.y, 500)

    def test_addcontrol(self):
        ret = self.aggregate_control.add_control(self.controls[0])
        self.assertEqual(ret, 1)
        ret = self.aggregate_control.add_control(self.controls[1])
        self.assertEqual(ret, 2)
        ret = self.aggregate_control.add_control(self.controls[2])
        self.assertEqual(ret, 3)

    # def test_toxml(self):
    #     for c in self.controls:
    #         self.aggregate_control.add_control(c)
    #     self.assertEqual(self.aggregate_control.to_xml(),
    #         '<control name="ZmllbGQy" type="labelv" color="gray" x="10" y="15" w="10" h="10"/>\n'
    #         '<control name="ZmllbGQz" type="labelv" color="gray" x="10" y="15" w="10" h="10"/>\n'
    #         '<control name="ZmllbGQw" type="labelv" color="gray" x="10" y="15" w="10" h="10"/>\n'
    #         '<control name="ZmllbGQx" type="labelv" color="gray" x="10" y="15" w="10" h="10"/>\n'
    #         '<control name="ZmllbGQ0" type="labelv" color="gray" x="10" y="15" w="10" h="10"/>')

    def test_width(self):
        """
        Width should return the width from the origin to the right edge of the
        rightmost control
        """
        for c in self.controls:
            self.aggregate_control.add_control(c)
        # all of the controls in self.controls go from origin to x=100
        self.assertEqual(self.aggregate_control.width, 100)

        # add another control that goes to 110
        c = controls.control_factory("led", "led", x=100, width=10)

        self.aggregate_control.add_control(c)
        self.assertEqual(self.aggregate_control.width, 110)

    def test_height(self):
        for c in self.controls:
            self.aggregate_control.add_control(c)
        self.assertEqual(self.aggregate_control.height, 100)     

class Test_DisplayList(unittest.TestCase):
    def setUp(self):
        pass

    def test_constructor(self):
        dl = DisplayList("list", 5, 100, 100)

    def test_constructor_badWidth(self):
        with self.assertRaises(ValueError) as cm:
            dl = DisplayList("list", 6, 100, 100)
        self.assertEqual(cm.exception.message, 
                         "Width must be evenly divisible by length")

class Test_Layout_DisplayList(unittest.TestCase):

    def test_createLayout(self):
        l = layout.Layout.createEmpty()
        tp = Tabpage()
        tp.name="TextDemo"

        dl = DisplayList("list", 10, 500, 500)
        dl.add_to_tabpage(tp)
        l.addTabpage(tp)

        l.writeToFile("/tmp", "test.touchosc", True)


if __name__ == "__main__":
    unittest.main()