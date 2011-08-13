#!/usr/bin/env python
__author__ = 'Michael Carroll <carroll.michael@gmail.com>'

import pybonjour
import select
import socket
import threading

def defaultDebugMsgCallback(msg):
    print "OSC_Bonjour Debug: ", msg

def defaultInfoMsgCallback(msg):
    print "OSC_Bonjour Info: ", msg

def defaultErrorMsgCallback(msg):
    print "OSC_Bonjour Error: ", msg


class osc_bonjour():
    def __init__(self,name,port):
        self.debug = defaultDebugMsgCallback;
        self.info = defaultInfoMsgCallback;
        self.error = defaultErrorMsgCallback;

        self.name = name
        self.port = port
        self.regtype = '_osc._udp'
        self.timeout = 5
        self.queried = []
        self.resolved = []


    def run_browser(self):
        browse_sdRef = pybonjour.DNSServiceBrowse(regtype = self.regtype,
                                                       callBack = self.browse_callback)
        try:
            try:
                while True:
                    ready = select.select([browse_sdRef],[],[])
                    if browse_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(browse_sdRef)
            except KeyboardInterrupt:
                pass
        finally:
            browse_sdRef.close()

    def run_register(self):
        reg_sdRef = pybonjour.DNSServiceRegister(name = self.name,
                                                regtype = self.regtype,
                                                port = self.port,
                                                callBack = self.register_callback)
        try:
            try:
                while True:
                    ready = select.select([reg_sdRef],[],[])
                    if reg_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(reg_sdRef)
            except KeyboardInterrupt:
                pass
        finally:
            reg_sdRef.close()


    def register_callback(self, sdRef, flags, errorCode, name, regtype, domain):
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.info("Registered %s service at %s"%(regtype,name))

    def query_record_callback(self, sdRef, flags, interfaceIndex, errorCode,
                              fullname, rrtype, rrclass, rdata, ttl):
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.info(" IP = %s"%(socket.inet_ntoa(rdata)))
            self.queried.append(True)

    def resolve_callback(self, sdRef, flags, interfaceIndex, errorCode, fullname,
                         hosttarget, port, txtRecord):
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.info("Resolved Service:")
            query_sdRef = \
                    pybonjour.DNSServiceQueryRecord(interfaceIndex = interfaceIndex,
                                                    fullname = hosttarget,
                                                    rrtype = pybonjour.kDNSServiceType_A,
                                                    callBack = self.query_record_callback)

            try:
                while not self.queried:
                    ready = select.select([query_sdRef],[],[],self.timeout)
                    if query_sdRef not in ready[0]:
                        self.info("Query record timed out")
                        break
                    pybonjour.DNSServiceProcessResult(query_sdRef)
                else:
                    self.queried.pop()
            finally:
                query_sdRef.close()

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

if __name__ == "__main__":
    asdf = osc_bonjour("Test Service",1234)
    t = threading.Thread(target=asdf.run_register)
    t.start()
    asdf.run_browser()
    t.join()
