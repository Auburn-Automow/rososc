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
import copy
from types import *


def defaultDebugMsgCallback(msg):
    """
    Default handler for bonjour.debug.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as a debug message.
    """
    print "Bonjour Debug: ", msg

def defaultInfoMsgCallback(msg):
    """
    Default handler for bonjour.info.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as an info message.
    """
    print "Bonjour Info: ", msg

def defaultErrorMsgCallback(msg):
    """
    Default handler for bonjour.error.  Can be overridden with the logging facility 
    of your choice

    @type msg: str
    @param msg: Message to be displayed as an error message.
    """
    print "Bonjour Error: ", msg

def quietHandler(msg):
    """
    Can be used as a message handler to silence command line output.

    @type msg: str
    @param msg: message that will be discarded silently.
    """
    return

class bonjour():
    """
    Wraps the pybonjour package to provide helper functions for registering a 
    Bonjour service and browsing the network for services matching a certain 
    regtype.
    """
    def __init__(self,name,port,regtype):
        """
        Initialize a Bonjour object.  

        @type name: UTF-8 String
        @param name: The name of the service to be advertised
        @type port: 16-bit unsigned integer
        @param port: The port of the service to be advertised
        @type regtype: str
        @param regtype: An mDNS-compliant registration type.  The service type 
                        followed by the protocol, separated by a dot (e.g. "_osc.
                        _udp").  A list of service types is available at:
                        U{http://www.dns-sd.org/ServiceTypes.html}
        """
        self.debug = defaultDebugMsgCallback;
        self.info = defaultInfoMsgCallback;
        self.error = defaultErrorMsgCallback;

        assert type(name) is StringType
        self.name = name
        assert type(port) is StringType or IntType
        self.port = int(port)
        assert type(regtype) is StringType
        self.regtype = regtype
        self.domain = "local"
        self.txtrecord = pybonjour.TXTRecord()
        self.txtrecord['Service'] = 'ROS OSC Service'
        self.fullname = pybonjour.DNSServiceConstructFullName(self.name,
                                                              self.regtype,
                                                              'local.')
        # Sometimes the fullname doesn't come out with a trailing period. This will 
        # cause comparisons in the browse/resolve/query callbacks to fail.
        if not self.fullname.endswith(u'.'):
            self.fullname += u'.'

        self.timeout = 2
        self.queried = []
        self.resolved = [] 
        self._isBrowserRunning = False
        self._isRegisterRunning = False
        #: Dictionary of clients detected by the Bonjour browser.  The browser
        # will maintain a list of the clients that are currently active, and will
        # prune clients as they leave the network.
        self.clients = dict()
        #: Lock for modifying the dictionary of clients.
        self.clientLock = threading.Lock()

    def setDebug(self,logFunction):
        """
        Set the debug logging handler

        @type logFunction: function 
        @param logFunction: Logging function handler of prototype f(msg)
        """
        assert type(logFunction) is FunctionType,\
            "Cannot override logger, %s is not of type function"%logFunction
        self.debug = logFunction

    def setInfo(self,logFunction):
        """
        Set the info logging handler

        @type logFunction: function 
        @param logFunction: Logging function handler of prototype f(msg)
        """
        assert type(logFunction) is FunctionType,\
            "Cannot override logger, %s is not of type function"%logFunction
        self.info = logFunction

    def setError(self,logFunction):
        """
        Set the error logging handler

        @type logFunction: function 
        @param logFunction: Logging function handler of prototype f(msg)
        """
        assert type(logFunction) is FunctionType,\
                "Cannot override logger, %s is not of type function"%logFunction
        self.error = logFunction

    def getClients(self):
        """
        Get the current list of clients active on the network

        @rtype: dict
        @return: List of clients currently active on the network
        """
        with self.clientLock:
            return copy.copy(self.clients)
        
    def run_browser(self):
        """
        Run the Bonjour service browser
        """
        if not self._isBrowserRunning:
            self._isBrowserRunning = True
            self.browser_t = threading.Thread(target=self.browser)
            self.browser_t.start()

    def stop_browser(self):
        """
        Stop the Bonjour service browser
        """
        if self._isBrowserRunning:
            self._isBrowserRunning = False
            self.browser_t.join()
            del self.browser_t

    def run_register(self):
        """
        Run the Bonjour service registration
        """
        if not self._isRegisterRunning:
            self._isRegisterRunning = True
            self.register_t= threading.Thread(target=self.register)
            self.register_t.start()

    def stop_register(self):
        """
        Stop the Bonjour service registration
        """
        if self._isRegisterRunning:
            self._isRegisterRunning = False
            self.register_t.join()
            del self.register_t

    def run(self):
        """
        Run both the worker threads for registration and browsing
        """
        self.run_browser()
        self.run_register()

    def shutdown(self):
        """
        Stop both the worker threads for registration and browsing
        """
        self.stop_browser()
        self.stop_register()

    def browser(self):
        """
        Routine for browsing the network for matching clients of type "regtype"
        """
        browse_sdRef = pybonjour.DNSServiceBrowse(regtype = self.regtype,
                                                       callBack = self.browse_callback)
        self.debug("Browser Service Started")
        try:
            try:
                while self._isBrowserRunning:
                    ready = select.select([browse_sdRef],[],[],self.timeout)
                    if browse_sdRef in ready[0]:
                        pybonjour.DNSServiceProcessResult(browse_sdRef)
            except Exception: 
                self.error("Exception in Browser")
                pass
        finally:
            browse_sdRef.close()
            self.debug("Browser Service Stopped")

    def register(self):
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
                while self._isRegisterRunning:
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
            if not fullname.endswith(u'.'):
                fullname += u'.'
            with self.clientLock:
                if self.clients.has_key(fullname.decode('utf-8')):
                    self.clients[fullname.decode('utf-8')]["ip"] = socket.inet_ntoa(rdata)
                else:
                    self.debug("Client not found")
            self.queried.append(True)

    def removed_callback(self, sdRef, flags, interfaceIndex, errorCode, fullname,
                         hosttarget, port, txtRecord):
        """
        Callback for removing hosts that have been detected through the browse 
        routine.
        """
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            with self.clientLock:
                if self.clients.has_key(hosttarget.decode('utf-8')):
                    del self.clients[hosttarget.decode('utf-8')]

   
    def resolve_callback(self, sdRef, flags, interfaceIndex, errorCode, fullname,
                         hosttarget, port, txtRecord):
        """
        Callback for resolving hosts that have been detected through the browse 
        routine.
        """
        if errorCode == pybonjour.kDNSServiceErr_NoError:
            if self.fullname == fullname:
                localhost = True
                self.debug("Resolved Self")
            else:
                localhost = False
            with self.clientLock:
                if not self.clients.has_key(hosttarget.decode('utf-8')) and not localhost:
                    self.clients[hosttarget.decode('utf-8')] = {"port":port}

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
            self.debug("Service Removed %s"%(serviceName))
            resolve_sdRef = pybonjour.DNSServiceResolve(0,
                                                        interfaceIndex,
                                                        serviceName,
                                                        regtype,
                                                        replyDomain,
                                                        self.removed_callback)
            try:
                while not self.resolved:
                    ready = select.select([resolve_sdRef],[],[],self.timeout)
                    if resolve_sdRef not in ready[0]:
                        self.error("Remove resolve timed out")
                        break
                    pybonjour.DNSServiceProcessResult(resolve_sdRef)
                else:
                    self.resolved.pop()
            finally:
                resolve_sdRef.close()
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
    import time
    try:
        while True:
            time.sleep(5)
            print osc_bonjour.getClients()
            pass
    except KeyboardInterrupt:
        osc_bonjour.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    import sys
    from optparse import OptionParser

    main(sys.argv,sys.stdout)
