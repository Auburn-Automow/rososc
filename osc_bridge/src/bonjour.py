#!/usr/bin/env python

"""

Python helper class for interfacing with Bonjour.

bonjour.py provides a simple way of advertising a service on a known registration
type and then also browses the current network to find clients of the same 
registration type.

"""

__author__ = 'Michael Carroll <carroll.michael@gmail.com>'

import pybonjour
import select
import socket
import threading


def defaultDebugMsgCallback(msg):
    """
    Default handler for bonjour.debug.  Can be overridden with the logging facility 
    of your choice
    """
    print "Bonjour Debug: ", msg

def defaultInfoMsgCallback(msg):
    """
    Default handler for bonjour.info.  Can be overridden with the logging facility 
    of your choice
    """
    print "Bonjour Info: ", msg

def defaultErrorMsgCallback(msg):
    """
    Default handler for bonjour.error.  Can be overridden with the logging facility 
    of your choice
    """
    print "Bonjour Error: ", msg

def quietHandler(msg):
    """
    Used to silence command line output.
    """
    return

class bonjour():
    """
    Bonjour class

    Wraps the pybonjour package to provide a simplified interface for registration
    and browsing/resolving similar clients on the network
    """
    def __init__(self,name="Test",port=1234,regtype='_osc._udp'):
        """
        Initialize a Bonjour object.  

            name:
                Name of the service to be advertised, must be UTF-8 string
            port:
                Port of the advertised service.  This is the port that clients will
                connect to.
            regtype:
                An mDNS-compliant registration type. The service type followed by 
                the protocol, separated by a dot (e.g. "_osc._udp").  A list of 
                service types is available at:
                <http://www.dns-sd.org/ServiceTypes.html>
        """

        self.debug = defaultDebugMsgCallback;
        self.info = defaultInfoMsgCallback;
        self.error = defaultErrorMsgCallback;

        self.name = name
        self.port = int(port)
        self.regtype = regtype
        self.domain = "local"
        self.fullname = pybonjour.DNSServiceConstructFullName(self.name,
                                                              self.regtype,
                                                              'local.')
        if not self.fullname.endswith(u'.'):
            self.fullname += u'.'

        self.timeout = 2
        self.queried = []
        self.resolved = [] 
        self._isRunning = False
        self.clients = dict()
        
    def run(self):
        """
        Spawn the worker threads for registration and browsing
        """
        self._isRunning = True
        self.browser = threading.Thread(target=self.run_browser)
        self.register = threading.Thread(target=self.run_register)

        self.browser.start()
        self.register.start()

    def shutdown(self):
        """
        Stop the worker threads for registration and browsing
        """
        self._isRunning = False
        self.browser.join()
        self.register.join()
        del self.browser, self.register

    def run_browser(self):
        """
        Routine for browsing the network for matching clients of type "regtype"
        """
        browse_sdRef = pybonjour.DNSServiceBrowse(regtype = self.regtype,
                                                       callBack = self.browse_callback)
        self.debug("Browser Service Started")
        try:
            try:
                while self._isRunning:
                    ready = select.select([browse_sdRef],[],[],self.timeout)
                    if browse_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(browse_sdRef)
            except Exception: 
                self.error("Exception in Browser")
                pass
        finally:
            browse_sdRef.close()
            self.debug("Browser Service Stopped")

    def run_register(self):
        """
        Register a service on the network for type "regtype"
        """
        reg_sdRef = pybonjour.DNSServiceRegister(name = self.name,
                                                regtype = self.regtype,
                                                port = self.port,
                                                callBack = self.register_callback)
        self.debug("Registration Service Started")
        try:
            try:
                while self._isRunning:
                    ready = select.select([reg_sdRef],[],[],self.timeout)
                    if reg_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(reg_sdRef)
            except Exception:
                self.error("Exception in Registration")
                pass
        finally:
            reg_sdRef.close()
            self.debug("Registration Service Stopped")


    def register_callback(self, sdRef, flags, errorCode, name, regtype, domain):
        """
        Callback used by the run_regsiter routine.
        """
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.info("Bonjour Service Registered at %s"%(self.fullname))

    def query_record_callback(self, sdRef, flags, interfaceIndex, errorCode,
                              fullname, rrtype, rrclass, rdata, ttl):
        """
        Callback for querying hosts IP addresses that come from the resolution
        routine
        """
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            self.debug(" IP = %s"%(socket.inet_ntoa(rdata)))
            self.queried.append(True)

    def resolve_callback(self, sdRef, flags, interfaceIndex, errorCode, fullname,
                         hosttarget, port, txtRecord):
        """
        Callback for resolving hosts that have been detected through the browse 
        routine.
        """
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            if self.fullname == fullname:
                self.debug("Resolved Self")
                return

            query_sdRef = \
                    pybonjour.DNSServiceQueryRecord(interfaceIndex = interfaceIndex,
                                                    fullname = hosttarget,
                                                    rrtype = pybonjour.kDNSServiceType_A,
                                                    callBack = self.query_record_callback)

            try:
                while not self.queried:
                    ready = select.select([query_sdRef],[],[],self.timeout)
                    if query_sdRef not in ready[0]:
                        self.error("Query record timed out")
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
        """
        Callback for browsing hosts of type "regtype" on the network.
        """
        if errorCode != pybonjour.kDNSServiceErr_NoError:
            return
        if not (flags & pybonjour.kDNSServiceFlagsAdd):
            self.debug("Service Removed")
            return

        self.debug("Service Added, Resolving")

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
                    self.error("Resolve timed out")
                    break
                pybonjour.DNSServiceProcessResult(resolve_sdRef)
            else:
                self.resolved.pop()
        finally:
            resolve_sdRef.close()


def main(argv, stdout):
    """
    Main function for when the script gets called on the command line.  Takes a 
    series of command line arguments and then starts a registration service.

    Mainly for debugging purposes.
    """
    parser = OptionParser()
    parser.add_option("-p","--port",action="store",type="int", dest="port",
            default=1234,
            help="Port of the service advertised on Bonjour")
    parser.add_option("-n","--name",action="store",type="string", dest="name",
            default="Test Bonjour Service",
            help="Name of the service advertised on Bonjour")
    parser.add_option("-r","--regtype",action="store",type="string",dest="regtype",
            default="_osc._udp",
            help="Registration type of the service advertised on Bonjour")
    parser.add_option("-v",action="count", dest="verbosity",
            help="Set verbosity level, up to -vv")
    (options, args) = parser.parse_args(argv)

    osc_bonjour = bonjour(name=options.name,
                          port=options.port,
                          regtype=options.regtype)
    # Set up the verbosity levels
    if options.verbosity == None:
        osc_bonjour.info = quietHandler
        osc_bonjour.debug = quietHandler
    elif options.verbosity == 1:
        osc_bonjour.debug = quietHandler

    osc_bonjour.run()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        osc_bonjour.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    import sys
    from optparse import OptionParser

    main(sys.argv,sys.stdout)
