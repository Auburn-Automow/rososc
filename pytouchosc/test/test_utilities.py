import unittest
import os
import sys

sys.path.append(os.path.abspath('../src'))

import pytouchosc.utilities
from pytouchosc.layout import Layout

layout_path = os.path.abspath('./layouts')

layout_ipod_h = Layout.createFromExisting(os.path.join(layout_path, 'test_ipod_h.touchosc'))
layout_ipod_v = Layout.createFromExisting(os.path.join(layout_path, 'test_ipod_v.touchosc'))
layout_ipad_h = Layout.createFromExisting(os.path.join(layout_path, 'test_ipad_h.touchosc'))
layout_ipad_v = Layout.createFromExisting(os.path.join(layout_path, 'test_ipad_v.touchosc'))
layout_five_tabpages = Layout.createFromExisting(os.path.join(layout_path, 'test_five_tabpages.touchosc'))


merge = pytouchosc.utilities.merge_layouts

class TestUtilities_MergeLayouts(unittest.TestCase):
	def setUp(self):
		pass

	def testWithWrongInput_NonList(self):
		with self.assertRaises(ValueError) as cm:
			merge(1.0)
		self.assertEqual(cm.exception.message, "Input argument layouts is not a list.")

	def testWithWrongInput_ListWithoutTabpages(self):
		with self.assertRaises(ValueError) as cm:
			merge([layout_ipod_h, 1.0])
		self.assertEqual(cm.exception.message, "Input list does not contain layouts")

	def testWithWrongInput_MixedMode(self):
		with self.assertRaises(ValueError) as cm:
			merge([layout_ipod_h, layout_ipad_h])
		self.assertEqual(cm.exception.message, "Input list contains layouts with different modes")

	def testWithWrongInput_MixedOrientation(self):
		with self.assertRaises(ValueError) as cm:
			merge([layout_ipod_h, layout_ipod_v])
		self.assertEqual(cm.exception.message, "Input list contains layouts with different orientations")


def rostest():
    suite = []
    suite.append(['MergeUtilities', TestUtilities_MergeLayouts])
    return suite

if __name__ == "__main__":
	unittest.main()