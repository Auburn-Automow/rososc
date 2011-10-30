import roslib; roslib.load_manifest('touchosc_bridge')

import rospy

from txosc import osc
from txosc import dispatch
from txosc import async

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from abstracttabpage import AbstractTabpageHandler

class DiagnosticsTabpageHandler(AbstractTabpageHandler):
    def init(self, layoutName):
        super(DiagnosticsTab, self).__init__(layoutName, "diagnostics")
        self.tabpageName = "diagnostics"
        self.msgDict = dict()
        
        self.diagSub = rospy.Subscriber("/diagnostics", self.diag_cb)
        
    def diag_cb(self, msg):
        for status in msg.status:
            self.msgDict[status.name] = status
        print msg
        
    