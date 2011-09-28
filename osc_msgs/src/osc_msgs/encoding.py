import roslib; roslib.load_manifest('osc_msgs')

from osc_msgs.msg import OSC
from osc_msgs.msg import IntParameter, BoolParameter 
from osc_msgs.msg import StrParameter, FloatParameter

def encode_osc(messageDict, client = None):
    msg = OSC()
    for k,v in messageDict.items():
        if   type(v) == int:   msg.ints.append(IntParameter(k,v))
        elif type(v) == bool:  msg.bools.append(BoolParameter(k,v))
        elif type(v) == str:   msg.strs.append(StrParameter(k,v))
        elif type(v) == float: msg.floats.append(FloatParameter(k,v))
    return msg

def decode_osc(msg):
    return dict([(kv.name, kv.value) for kv in msg.bools + msg.ints + msg.strs + msg.floats])
