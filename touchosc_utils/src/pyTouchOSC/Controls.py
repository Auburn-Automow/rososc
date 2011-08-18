# -*- coding: utf-8 -*-
# Controls.py
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

from lxml import etree
import base64

class Control(etree.ElementBase):
    colors = ["red","green","blue","yellow","purple","gray","orange"]

    @apply
    def name():
        doc="""Control Name"""
        def fget(self):
            return base64.b64decode(self.get('name'))
        def fset(self,value):
            self.set('name',base64.b64encode(str(value).encode('utf-8')))
        return property(**locals())

    @apply
    def x():
        doc="""X Position"""
        def fget(self):
            return self.get('x')
        def fset(self,value):
            self.set('x',value)
        return property(**locals())

    @apply
    def y():
        doc="""Y Position"""
        def fget(self):
            return self.get('y')
        def fset(self,value):
            self.set('y',value)
        return property(**locals())

    @apply
    def width():
        doc="""Control Width"""
        def fget(self):
            return self.get('w')
        def fset(self,value):
            self.set('w',value)
        return property(**locals())

    @apply
    def height():
        doc="""Control Height"""
        def fget(self):
            return self.get('h')
        def fset(self,value):
            self.set('h',value)
        return property(**locals())

    @apply
    def color():
        doc="""Control Color"""
        def fget(self):
            return self.get('color')
        def fset(self,value):
            value = value.lower()       #Check the case, to be sure.
            assert any(value == val for val in self.colors), \
                "%s is not a valid color name"%value
            self.set('color',value)
        return property(**locals())

class TextField(Control):
    @apply
    def textSize():
        doc="""Label Font Size"""
        def fget(self):
            return self.get('size')
        def fset(self,value):
            return self.set('size',value)
        return property(**locals())

    @apply
    def background():
        doc="""Background Enabled"""
        def fget(self):
            return self.get('background')
        def fset(self,value):
            assert type(value) is bool, "Background must be boolean"
            self.set('background',str(value).lower)
        return property(**locals())

    @apply
    def outline():
        doc="""Outline Enabled"""
        def fget(self):
            return self.get('outline')
        def fset(self,value):
            assert type(value) is bool, "Outline must be boolean"
            self.set('outline',str(value).lower)
        return property(**locals())


class Label(TextField):
    @apply
    def text():
        doc="""Label Text"""
        def fget(self):
            return base64.b64decode(self.get('text'))
        def fset(self,value):
            self.set('text',base64.b64encode(str(value).encode('utf-8')))
        return property(**locals())

class Time(TextField):
    @apply
    def seconds():
        doc="""Display seconds on time control"""
        def fget(self):
            return self.get('seconds')
        def fset(self,value):
            assert type(value) is bool, "Seconds must be boolean"
            self.set('seconds',str(value).lower)
        return property(**locals())

class ScalableControl(Control):
    @apply
    def scalef():
        doc="""Bottom value on the control's scale"""
        def fget(self):
            return self.get('scalef')
        def fset(self,value):
            self.set('scalef',value)
        return property(**locals())

    @apply
    def scalet():
        doc="""Top value on the control's scale"""
        def fget(self):
            return self.get('scalet')
        def fset(self,value):
            self.set('scalet',value)
        return property(**locals())

class Encoder(ScalableControl):
    pass

class LED(ScalableControl):
    pass

class Button(ScalableControl):
    pass

class MultiButton(Button):
    pass

class XYPad(ScalableControl):
    pass

class Dial(ScalableControl):
    pass

class MultiDial(Dial):
    pass




type_class_mapping = {
                "led":          LED,
                "labelv":       Label,
                "labelh":       Label,
                "push":         Button,
                "toggle":       Button,
                "xy":           XYPad,
                "faderv":       Dial,
                "faderh":       Dial,
                "rotaryv":      Dial,
                "rotaryh":      Dial,
                "encoder":      Encoder,
                "batteryv":     TextField,
                "batteryh":     TextField,
                "timev":        Time,
                "timeh":        Time,
                "multipush":    MultiButton,
                "multitoggle":  MultiButton,
                "multifaderv":  MultiDial,
                "multifaderh":  MultiDial 
                }

lookup = etree.AttributeBasedElementClassLookup('type',type_class_mapping)
layoutParser = etree.XMLParser(remove_blank_text=True)
parser=etree.XMLParser()
parser.setElementClassLookup(lookup)

tree = etree.parse("index.xml",parser)

root = tree.getroot()
