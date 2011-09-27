# -*- coding: utf-8 -*-
# layout_sync
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


import bonjour

import sys
import socket
import zipfile
import os
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

def make_layoutHandler_class(layoutName,layoutFile):
    """
    Function to generate a class that extends BaseHTTPRequest Handler.

    @type layoutName: str
    @param layoutName: Name of the layout file (e.g. sample.touchosc)
    @type layoutFile: File
    @param layoutFile: The layout file to be loaded to the TouchOSC app.
    """
    #TODO: Do HTTPRequestHandler the proper "Python way".
    # This is sort of a hack.  The way that this should probably be done:
    # 1. Extend the HTTPServer class to take "layoutFile" as part of the constructor.
    # 2. In my overridden do_GET function in the LayoutHTTPRequestHandler, I should
    # then take advantage of the instance variable "server" which points to the 
    # over ridden HTTPServer class.  I should be able to get to all of the instance
    # variables of the HTTPServer class through that "server" variable.
    #
    # That is what I tried, but couldn't get working.  Because this is not a really
    # mission-critical application, this will work for now.
    class LayoutHTTPRequestHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            try:
                self.send_response(200)
                self.send_header('Content-type','application/touchosc')
                self.send_header('Content-Disposition',
                                 'attachment; filename="%s"'%layoutName)
                self.end_headers()
                self.wfile.write(layoutFile)
            except Exception:
                import traceback
                traceback.print_exc(f=sys.stdout)   
            return
    return LayoutHTTPRequestHandler

class LayoutServer(object):
    def __init__(self,layoutPath,name,port):
        if not os.path.lexists(layoutPath):
            raise ValueError("Layout file not found: %s",layoutPath)
        layoutZip = zipfile.ZipFile(layoutPath,"r")
        layoutFile = layoutZip.read("index.xml")
        layoutName = os.path.basename(layoutPath)
        self.bonjourServer = bonjour.Bonjour(name,port,
                                             '_touchosceditor._tcp')
        self.httpServer = HTTPServer(('',port),
                                     make_layoutHandler_class(layoutName,
                                                              layoutFile))

    def run(self):
        self.bonjourServer.run_register()
        self.httpServer.serve_forever()
    
    def stop(self):
        self.httpServer.socket.close()
        self.bonjourServer.stop_register()
        pass

def main(argv,stdout):
    usage = "usage: %prog [options] /path/to/layout.touchosc"
    parser = OptionParser(usage=usage)
    parser.add_option("-p","--port",action="store",type="int",dest="port",
            default=9658,
            help="Port that the layout server will host on.")
    parser.add_option("-n","--name",action="store",type="string",dest="name",
            default="OSC Layout Server on %s"%socket.gethostname(),
            help="Name that will appear in TouchOSC's Server List")
    (options,args) = parser.parse_args(argv)
    if len(args) < 2:
        parser.error("Please specify a layout file.")
        sys.exit(1)
    layoutFilePath = args[1];    
    try:
        server = LayoutServer(layoutFilePath,options.name,options.port)
    except ValueError:
        parser.error("Layout file not found: %s",layoutFilePath)
        sys.exit(1)
    
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()

if __name__ == '__main__':
    from optparse import OptionParser
    main(sys.argv, sys.stdout)
