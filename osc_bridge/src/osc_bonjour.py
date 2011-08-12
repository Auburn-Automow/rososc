#!/usr/bin/env python
__author__ = 'Michael Carroll <carroll.michael@gmail.com>'

import pybonjour
import select
import socket

def defaultDebugMsgCallback(msg):
    print "OSC_Bonjour Debug: ", msg

def defaultInfoMsgCallback(msg):
    print "OSC_Bonjour Info: ", msg

def defaultErrorMsgCallback(msg):
    print "OSC_Bonjour Error: ", msg


class osc_bonjour():
    def __init__(self, name, port):
        self.debug = defaultDebugMsgCallback;
        self.info = defaultInfoMsgCallback;
        self.error = defaultErrorMsgCallback;

        self.regtype = '_osc._udp'
        self.timeout = 5
        self.queried = []
        self.resolved = []

        self.reg_sdRef = pybonjour.DNSServiceRegister(name = name,
                                                  regtype = self.regtype,
                                                  port = port,
                                                  callBack = self.register_callback)

        self.browse_sdRef = pybonjour.DNSServiceBrowse(regtype = self.regtype,
                                                       callBack = self.browse_callback)


    def register_callback(self, sdRef, flags, errorCode, name, regtype, domain):
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.info("Bonjour Service Registered")
        else:
            self.error("Register failed with code: %s"%errorCode)


    def resolve_callback(self, sdRef, flags, interfaceIndex, errorCode, fullname,
                         hosttarget, port, txtRecord):
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            ip = socket.gethostbyname(hosttarget)
            self.info("Resolved Service:")
            self.info(" host = %s (%s)"%(hosttarget, ip))
            self.info(" port = %s"%port)
            
            self.resolved.append(True)
        else:
            self.error("Resolve failed with code: %s"%errorCode)


    def browse_callback(self, sdRef, flags, interfaceIndex, errorCode, serviceName,
                        regtype, replyDomain):
        if errorCode != pybonjour.kDNSServiceErr_NoError:
            return
        if not (flags & pybonjour.kDNSServiceFlagsAdd):
            self.info("Service Removed")
            return

        self.info("Service Added, Resolving")

        resolve_sdRef = pybonjour.DNSServiceResolve(0,
                                                    interfaceIndex,
                                                    serviceName,
                                                    regtype,
                                                    replyDomain,
                                                    self.resolve_callback)

        try:
            while not self.resolved:
                ready = select.select([resolve_sdRef],[],[],self.timeout)
                if resolve_sdRef not in ready[0]:
                    self.info("Resolve timed out")
                    break
                pybonjour.DNSServiceProcessResult(resolve_sdRef)
            else:
                self.resolved.pop()
        finally:
            resolve_sdRef.close()


    def run(self):
        try:
            try:
                while True:
                    ready = select.select([self.reg_sdRef],[],[])
                    if self.reg_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(self.reg_sdRef)
                    ready = select.select([self.browse_sdRef],[],[])
                    if self.browse_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(self.browse_sdRef)
            except KeyboardInterrupt:
                pass
        finally:
            self.reg_sdRef.close()
            self.browse_sdRef.close()

if __name__ == "__main__":
    asdf = osc_bonjour('Test Service',9000)
    asdf.run()
