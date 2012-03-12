import unittest
import os
import sys

from StringIO import StringIO
from zipfile import ZipFile
from lxml import etree

sys.path.append(os.path.abspath('../../'))
from pytouchosc.layout import Layout

layoutPath = os.path.abspath('./layouts')
layoutFile = 'ROS-Demo-iPad.touchosc'
layoutBare = 'index.xml'

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
        if os.path.exists(self.testJoinPath):
            os.remove(self.testJoinPath)

    def test_writeToFile_badPath(self):
        with self.assertRaises(ValueError) as cm:
            self.layout.writeToFile('a','test.touchosc')
        self.assertEqual(cm.exception.message, "path does not exist: 'a'")

    def test_writeToFile_unwritablePath(self):
        # First check to make sure /root exists and is not writable
        if not os.path.isdir('/root'):
            self.fail("Test uses /root as a non-writable directory, /root doesn't exist")
        if os.access('/root', os.W_OK):
            self.fail("Test uses /root as a non-writable directory, /root is writable, stopping")
        
        with self.assertRaises(IOError) as cm:
            self.layout.writeToFile('/root', 'test')
        self.assertEqual(cm.exception.message, "Permission Denied: '/root'")

    def test_writeToFile_withExtension(self):
        self.layout.writeToFile(self.testPath, self.testFileExt)

        self.assertFalse(os.path.exists(self.testJoinPath + '.touchosc'))
        self.assertTrue(os.path.exists(self.testJoinPath), "Didn't create file")

    def test_writeToFile_withoutExtension(self):
        self.layout.writeToFile(self.testPath, self.testFile)

        self.assertFalse(os.path.exists(self.testJoinPath + '.touchosc'))
        self.assertTrue(os.path.exists(self.testJoinPath), "Didn't create file")

    def test_writeToFile_alreadyExists(self):
        with self.assertRaises(IOError) as cm:
            self.layout.writeToFile(self.layoutPath, self.layoutFile)

    def test_writeToFile_withReplacement(self):
        f = open(self.testJoinPath, 'w')
        f.close()

        self.layout.writeToFile(self.testPath, self.testFileExt, replace_existing=True)
        self.assertGreater(os.path.getsize(self.testJoinPath), 0)

    # def test_writeToFile(self):
    #     layout = Layout.createFromExisting(os.path.join(self.layoutPath, 'test_ipod_h.touchosc'))
    #     layout.writeToFile('/tmp', 'test.touchosc', True)