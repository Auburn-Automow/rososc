# -*- coding: utf-8 -*-
# TouchOSCLayout.py
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

class TouchOSCLayout(object):
    """
    docstring for TouchOSCLayout
    """
    def __init__(self, version=10, mode='iPad', orientation='horizontal'):
        self._layout = etree.Element("layout")
        self._layout.attrib['version'] = str(version)
        self._layout.attrib['mode'] = str(1) if 'iPad' else str(0)
        self._layout.attrib['orientation'] = orientation
        return

    @apply
    def version():
        doc="""docstring"""
        def fget(self):
            return self._layout.attrib['version']
        def fset(self, value):
            self._layout.attrib['version'] = str(value).encode('utf-8')
        return property(**locals())

    @apply
    def mode():
        doc="""docstring"""
        def fget(self):
            return self._layout.attrib['mode']
        def fset(self,value):
            self._layout.attrib['mode'] = str(value).encode('utf-8')
        return property(**locals())

    @classmethod
    def createFromExisting(source):
        """
        Create a TouchOSCLayout instance from an existing TouchOSC Layout.

        @type source: filename or fileobject 
        @param source: Path to an existing .touchosc file, or fileobject containing
            the layout XML data.
        @rtype: TouchOSCLayout
        @return: An instance containing the layout 
        """
        layoutParser = etree.XMLParser(remove_blank_text=True)
        if type(source) is str:
            try:
                f = ZipFile(source,"r")
                layoutTree = etree.parse(StringIO(f.read("index.xml")),layoutParser)
            except IOError:
                pass
        elif type(source) is file:
            #TODO: Test this
            try:
                layoutTree = etree.parse(source,layoutParser)
            except:
                pass
        layout = TouchOSCLayout()
        

        pass

