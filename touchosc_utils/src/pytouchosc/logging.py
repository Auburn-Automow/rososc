#!/usr/bin/env python

# -*- coding: utf-8 -*-
# bonjour.py

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

import sys

def logdebug(msg, *args):
    """
    Default handler.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as a debug message.
    @param args: format-string arguments, if necessary
    """
    sys.stdout.write("Debug: %s\n" % (msg % args))


def loginfo(msg, *args):
    """
    Default handler.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as an info message.
    @param args: format-string arguments, if necessary
    """
    sys.stdout.write("Info: %s\n" % (msg % args))

def logerror(msg, *args):
    """
    Default handler.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as an error message.
    @param args: format-string arguments, if necessary
    """
    sys.stderr.write("Error: %s\n" % (msg % args))

def logquiet(msg, *args):
    """
    Can be used as a message handler to silence command line output.

    @type msg: str
    @param msg: message that will be discarded silently.
    """
    return

