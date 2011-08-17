#!/usr/bin/env python
import roslib; roslib.load_manifest('osc_utils')

import bonjour

import sys
import socket
import os
import zipfile
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

def defaultError(msg,*args):
    """
    Default handler for error messages.  Can be overridden with the logging facility
    of your choice

    @type msg: str
    @param msg: Message to be displayed as an error message.
    @param args: format-string arguments, if necessary
    """
    sys.stderr.write("Layout Server Error: %s\n"%(msg%args))

def defaultInfo(msg,*args):
    """
    Default handler for messages.  Can be overridden with the logging facility of
    your choice.

    @type msg: str
    @param msg: Message to be displayed as a message.
    @param args: format-string arguments, if necessary
    """
    sys.stdout.write("Layout Server Info: %s\n"%(msg%args))

class layoutHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global layoutFile
        f = zipfile.ZipFile(layoutFile,"r")
        try:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.send_header('Content-Disposition','filename=Simple.touchosc')
            self.end_headers()
            self.wfile.write(f.read("index.xml"))
        except Exception:
            import traceback
            traceback.print_exc(f=sys.stdout)
        return


def main(argv, stdout):
    global layoutFile
    usage = "usage: %prog [options] /path/to/layout.osc"
    parser = OptionParser(usage=usage)
    parser.add_option("-p","--port",action="store",type="int",dest="port",
            default=8000,
            help="Port that the layout server will host on")
    parser.add_option("-n","--name",action="store",type="string",dest="name",
            default="OSC Layout Server on %s"%socket.gethostname(),
            help="Name that will appear in TouchOSC's Server List")

    (options,args) = parser.parse_args(argv)

    b = bonjour.bonjour(options.name,options.port,'_touchosceditor._tcp')
    b.run_register()
    server = HTTPServer(('',options.port),layoutHandler)

    layoutFile = args[1]

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
    b.stop_register()


if __name__ == '__main__':
    from optparse import OptionParser

    main(sys.argv, sys.stdout)

