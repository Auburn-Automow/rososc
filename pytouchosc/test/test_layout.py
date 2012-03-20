import unittest
import os
import sys

from StringIO import StringIO
from zipfile import ZipFile
from lxml import etree
import random

sys.path.append(os.path.abspath('../src'))
from pytouchosc.layout import Layout

from pytouchosc.tabpage import Tabpage

import pytouchosc.controls as controls

layoutPath = os.path.abspath('./layouts')
layoutFile = 'ROS-Demo-iPad.touchosc'
layoutBare = 'index.xml'

class LayoutTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutFile)
        self.layout = Layout.createFromExisting(self.path)
        
    def test_getNumberTabpages(self):
        self.assertEqual(self.layout.getNumberTabpages(),2)
        
    def test_getTabpageNames_base64(self):
        tabpageNames = self.layout.getTabpageNames(encoded=True)
        self.assertEqual(tabpageNames[0],"MQ==")
        self.assertEqual(tabpageNames[1],"VGV4dERlbW8=")
            
    def test_getTabpageNames_plainText(self):
        tabpageNames = self.layout.getTabpageNames()
        self.assertEqual(tabpageNames[0],'1')
        self.assertEqual(tabpageNames[1],'TextDemo')
  
    def test_addTabpage(self):
        tp = Tabpage()
        tp.name = "asdf"
        self.layout.addTabpage(tp)
        tabpageNames = self.layout.getTabpageNames()
        self.assertEqual(tabpageNames[0],'1')
        self.assertEqual(tabpageNames[1],'TextDemo')
        self.assertEqual(tabpageNames[2], "asdf")


    def test_walkDict(self):
        aDict = {'toggle1': {None: 0.0, 'z': False}}
        testDict = {'/toggle1': 0.0, '/toggle1/z': False}
        newDict = dict(self.layout.walkDict(aDict))
        self.assertEqual(newDict, testDict)
        
    def test_walkDict_withPath(self):
        aDict = {'toggle1': {None: 0.0, 'z': False}}
        testDict = {'/1/toggle1': 0.0, '/1/toggle1/z': False}
        path = '/1'
        newDict = dict(self.layout.walkDict(aDict, path))
        self.assertEqual(newDict, testDict)
        # Check for side effects
        self.assertEqual(path,'/1')

    def test_setOrientation(self):
        self.layout.orientation = 'vertical'
        self.assertEqual(self.layout.orientation, 'vertical')
        self.layout.orientation = 'horizontal'
        self.assertEqual(self.layout.orientation, 'horizontal')
        self.layout.orientation = 'vertical'
        self.assertEqual(self.layout.orientation, 'vertical')
        self.layout.orientation = 'horizontal'
        self.assertEqual(self.layout.orientation, 'horizontal')
        with self.assertRaises(ValueError) as cm:
            self.layout.orientation = 'bob'

    def test_str(self):
        self.assertEqual(str(self.layout),
            "ROS-Demo-iPad")
        self.layout.name = None
        self.assertEqual(str(self.layout),
            "Untitled-1-TextDemo")

    def test_repr(self):
        self.assertEqual(repr(self.layout), 
            ("Layout: ROS-Demo-iPad, Mode: iPod, "
                "Orientation: horizontal, Tabpages: ['1', 'TextDemo']"))


class LayoutTest_CreateFromExistingZip(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutFile)
        self.layout = Layout.createFromExisting(self.path)

    def testCreateFromExisting(self):
        self.assertIsInstance(self.layout,Layout,"Not an instance")
        
    def testCreateFromExistingMode(self):
        self.assertEqual(self.layout.mode,"1","Mode isn't iPad")
        
    def testCreateFromExistingOrientation(self):
        self.assertEqual(self.layout.orientation,"horizontal","Orientation wrong")
        
    def testCreateFromExistingVersion(self):
        self.assertEqual(self.layout.version,"10")

    def testCreateFromExistingName(self):
        self.assertEqual(self.layout.name, "ROS-Demo-iPad")

    def test_getNumberTabpages(self):
        self.assertEqual(self.layout.getNumberTabpages(), 2)

    def test_getTabpageNames(self):
        self.assertEqual(self.layout.getTabpageNames(), ['1', 'TextDemo'])

class LayoutTest_CreateFromExistingFile(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(layoutPath, layoutBare)
        self.layout = Layout.createFromExisting(self.path)

    def testCreateFromExisting(self):
        self.assertIsInstance(self.layout,Layout,"Not an instance")
        
    def testCreateFromExistingMode(self):
        self.assertEqual(self.layout.mode,"1","Mode isn't iPad")
        
    def testCreateFromExistingOrientation(self):
        self.assertEqual(self.layout.orientation,"horizontal","Orientation wrong")
        
    def testCreateFromExistingVersion(self):
        self.assertEqual(self.layout.version,"10")

    def testCreateFromExistingName(self):
        """Creating from an XML file should not set name"""
        self.assertEqual(self.layout.name, None)

    def test_getNumberTabpages(self):
        self.assertEqual(self.layout.getNumberTabpages(), 2)

    def test_getTabpageNames(self):
        self.assertEqual(self.layout.getTabpageNames(), ['1', 'TextDemo'])


class LayoutWriteTest(unittest.TestCase):
    def setUp(self):
        self.layoutFile = layoutFile
        self.layoutPath = layoutPath
        self.layoutJoinPath = os.path.join(self.layoutPath, self.layoutFile)

        self.layout = Layout.createFromExisting(self.layoutJoinPath)

        self.testFile = 'test'
        self.testFileExt = 'test.touchosc'
        self.testPath = layoutPath
        self.testJoinPath = os.path.join(self.testPath, self.testFileExt)

    def tearDown(self):
        """Clean up after ourselves if the file was written"""
        if os.path.exists(self.testJoinPath):
            os.remove(self.testJoinPath)

    def test_writeToFile_badPath(self):
        """Should fail on non-existant path"""
        with self.assertRaises(ValueError) as cm:
            self.layout.writeToFile('a','test.touchosc')
        self.assertEqual(cm.exception.message, "path does not exist: 'a'")

    def test_writeToFile_unwritablePath(self):
        """Should fail on an unwritable path"""
        # First check to make sure /root exists and is not writable
        if not os.path.isdir('/bin'):
            self.fail("/bin doesn't exist")
        if os.access('/bin', os.W_OK):
            self.fail("/bin is writable, stopping")
        
        with self.assertRaises(IOError) as cm:
            self.layout.writeToFile('/bin', 'test')
        self.assertEqual(cm.exception.message, "Permission Denied: '/bin'")

    def test_writeToFile_withExtension(self):
        """Should check to see if the argument already has the extension, and not add another"""
        self.layout.writeToFile(self.testPath, self.testFileExt)

        self.assertFalse(os.path.exists(self.testJoinPath + '.touchosc'))
        self.assertTrue(os.path.exists(self.testJoinPath), "Didn't create file")

    def test_writeToFile_withoutExtension(self):
        """Should check to see if the argument doesn't have the extension, and add it"""
        self.layout.writeToFile(self.testPath, self.testFile)

        self.assertFalse(os.path.exists(self.testJoinPath + '.touchosc'))
        self.assertTrue(os.path.exists(self.testJoinPath), "Didn't create file")

    def test_writeToFile_alreadyExists(self):
        """Raise an IOError if the file already exists"""
        with self.assertRaises(IOError) as cm:
            self.layout.writeToFile(self.layoutPath, self.layoutFile)

    def test_writeToFile_withReplacement(self):
        """Optional parameter to silence the IOError and go ahead and write the file."""
        f = open(self.testJoinPath, 'w')
        f.close()

        self.layout.writeToFile(self.testPath, self.testFileExt, replace_existing=True)
        self.assertGreater(os.path.getsize(self.testJoinPath), 0)

    def test_writeToFile_usingNameProperty(self):
        """Leaving off the name parameter should use the "name" property"""
        self.layout.name = self.testFile 
        self.layout.writeToFile(self.testPath)

        self.assertTrue(os.path.exists(self.testJoinPath), "Didn't create file")

    def test_writeToFile_noNameProperty(self):
        """Leaving off the name parameter should use the "name" property 
        If name isn't set, it should raise a ValueError"""
        self.layout.name = None
        with self.assertRaises(ValueError) as cm:
            self.layout.writeToFile(self.layoutPath)
        self.assertEqual(cm.exception.message, "Layout has no property: name, and filename parameter was not set")

class TestAllControls(unittest.TestCase):
    def randColor(self):
        colors = controls.Control.colors
        return colors[random.randint(0,len(colors)-1)]

    def test_allcontrols(self):
        l = Layout.createEmpty()
        tp = Tabpage()

        tp.name = "Demo"

        tp.append(controls.control_factory("led","led", color=self.randColor(), x=0, y=0))
        tp.append(controls.control_factory("labelv","labelv", color=self.randColor(), x=100, y=0))
        tp.append(controls.control_factory("labelh","labelh", color=self.randColor(), x=200, y=0))
        tp.append(controls.control_factory("push", "push", color=self.randColor(), x=300, y=0))
        tp.append(controls.control_factory("toggle", "toggle", color=self.randColor(), x=400, y=0))
        tp.append(controls.control_factory("xy", "xy", color=self.randColor(), x=500, y=0, width=200, height=200))

        tp.append(controls.control_factory("faderv","faderv", color=self.randColor(), x=0, y=100))
        tp.append(controls.control_factory("faderh","faderh", color=self.randColor(), x=100, y=100))
        tp.append(controls.control_factory("rotaryv","rotaryv", color=self.randColor(), x=200, y=100))
        tp.append(controls.control_factory("rotaryh","rotaryh", color=self.randColor(), x=300, y=100))
        tp.append(controls.control_factory("encoder", "encoder", color=self.randColor(), x=400, y=100))

        tp.append(controls.control_factory("batteryv","batteryv", color=self.randColor(), x=0, y=200))
        tp.append(controls.control_factory("batteryh","batteryh", color=self.randColor(), x=100, y=200))
        tp.append(controls.control_factory("timev","timev", color=self.randColor(), x=200, y=200))
        tp.append(controls.control_factory("timeh","timeh", color=self.randColor(), x=300, y=200))

        tp.append(controls.control_factory("multipush", "multipush", color=self.randColor(), x=0, y=300, width=300, height=300))
        tp.append(controls.control_factory("multitoggle", "multitoggle", color=self.randColor(), x=300, y=300, width=300, height=300))

        tp.append(controls.control_factory("multifaderh", "multifaderh", color=self.randColor(), x=0, y=600, width=300, height=300))
        tp.append(controls.control_factory("multifaderv", "multifaderv", color=self.randColor(), x=300, y=600, width=300, height=300))

        tp.append(controls.control_factory("multixy", "multixy", color=self.randColor(), x=600, y=300, height=600, width=150))

        l.addTabpage(tp)
        l.writeToFile("/home/mjcarroll", "Demo.touchosc", True)




def suite():
    load = unittest.TestLoader()
    suite = load.loadTestsFromTestCase(LayoutTest)
    suite.addTests(load.loadTestsFromTestCase(LayoutTest_CreateFromExistingZip))
    suite.addTests(load.loadTestsFromTestCase(LayoutTest_CreateFromExistingFile))
    suite.addTests(load.loadTestsFromTestCase(LayoutWriteTest))
    return suite

def rostest():
    suite = []
    suite.append(['LayoutTest', LayoutTest])
    suite.append(['CreateFromExistingFile', LayoutTest_CreateFromExistingFile])
    suite.append(['CreateFromExistingZip', LayoutTest_CreateFromExistingZip])
    suite.append(['WriteLayout', LayoutWriteTest])
    return suite

if __name__ == "__main__":
    unittest.main()
