# -*- coding: utf-8 -*-
# Layout.py
#
# Copyright (c) 2011, Michael Carroll
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "Michael Carroll <carroll.michael@gmail.com>"
__version__ = "1.7.3"

from zipfile import ZipFile
from lxml import etree
import base64
import os
from StringIO import StringIO

import controls, tabpage

class Layout(object):
    """
    docstring
    """
    
    # These dictionaries are probably largely unnecessary, but I'm putting them in
    # just in case Hexler decides to do a massive update at some point in the
    # future

    #: Dictionary of TouchOSC versions
    VERSION = {
            "current":str(10).encode('utf-8'), # Tracks current TouchOSC
            10:str(10).encode('utf-8'), # TouchOSC 1.7.3
            8:str(8).encode('utf-8')                # < 1.7.3
            }

    #: Dictionary of "modes", or target devices
    MODE = {
            "iPhone":str(0).encode('utf-8'),
            "iPad":str(1).encode('utf-8'),
            0:str(0).encode('utf-8'),
            1:str(1).encode('utf-8')
            }

    #: Dictionary of orientations
    ORIENTATION = {
            "horizontal":"horizontal".encode('utf-8'),
            "vertical":"vertical".encode('utf-8')
            }

    def __init__(self, tree):
        """
        Constructor for Layout

        @type tree: etree.ElementTree
        @param tree: ElementTree for the layout object to be instantiated 
        
        """
        self._layout = tree
        self._layoutRoot = tree.getroot()
        self._tabPages = dict()
        for tp in self._layoutRoot.getchildren():
            self._tabPages[tp.name] = tp

    def getNumberTabpages(self):
        return len(self._layoutRoot.getchildren())
    
    def getTabpageNames(self,encoded = False):
        if encoded:
            return [x.attrib['name'] for x in self._tabPages.itervalues()]
        else:
            return [x for x in self._tabPages.iterkeys()]
    
    def getSendableMessages(self, tabpage):
        return self._tabPages[tabpage].getSendDict()
    
    def getReceivableMessages(self, tabpage):
        return self._tabPages[tabpage].getReceiveDict()
        
    @apply
    def version():
        doc = """docstring"""
        def fget(self):
            return self._layoutRoot.attrib['version']
        def fset(self, value):
            self._layoutRoot.attrib['version'] = str(value).encode('utf-8')
        return property(**locals())

    @apply
    def mode():
        doc = """docstring"""
        def fget(self):
            return self._layoutRoot.attrib['mode']
        def fset(self, value):
            self._layoutRoot.attrib['mode'] = str(value).encode('utf-8')
        return property(**locals())

    @apply
    def orientation():
        doc = """docstring"""
        def fget(self):
            return self._layoutRoot.attrib['orientation']
        def fset(self, value):
            self._layoutRoot.attrib['orientation'] = str(value).encode('utf-8')
        return property(**locals())

    @classmethod
    def createFromExisting(cls, source):
        """
        Create a TouchOSCLayout instance from an existing TouchOSC Layout.

        @type source: filename or fileobject 
        @param source: Path to an existing .touchosc file, or fileobject containing
            the layout XML data.
        @rtype: Layout 
        @return: An instance containing the layout 
        """
        fallback = etree.ElementDefaultClassLookup()
        lookupTabpages = etree.ElementNamespaceClassLookup(fallback)
        namespace = lookupTabpages.get_namespace(None)
        namespace['tabpage'] = tabpage.Tabpage
        lookupControls = etree.AttributeBasedElementClassLookup('type', 
                                controls.type_class_mapping,lookupTabpages)
        layoutParser = etree.XMLParser(remove_blank_text=True)
        layoutParser.setElementClassLookup(lookupControls)
        
        if type(source) is str:
            try:
                f = ZipFile(source, "r")
                layoutTree = etree.parse(StringIO(f.read("index.xml")), layoutParser)
                f.close()
            except IOError:
                pass
        elif type(source) is file:
            #TODO: Test this
            try:
                layoutTree = etree.parse(source, layoutParser)
            except:
                pass
        return Layout(layoutTree)

    @classmethod
    def createEmpty(cls, version=VERSION["current"],
                mode=MODE['iPad'],
                orientation=ORIENTATION['horizontal']):
        """
        Create an empty Layout instance.

        A minimum layout must include a version, a mode, and an orientation.

        @type version: str
        @param version: One of the versions from Layout.VERSION.  It is generally 
            advisable to use VERSION["current"]
        @type mode: str
        @param mode: One of the modes from Layout.MODE.  Available modes are 
            currently "iPad" and "iPhone".
        @type orientation: str
        @param orientation: One of the orientations from Layout.ORIENTATION.  
            Available orientations are "horizontal" and "vertical"
        @rtype: Layout
        @return: An instance of the Layout class 
        """
        layoutTag = etree.Element("layout")
        layoutTag.attrib["version"] = version
        layoutTag.attrib["mode"] = mode
        layoutTag.attrib["orientation"] = orientation

        return Layout(etree.ElementTree(layoutTag))


    
