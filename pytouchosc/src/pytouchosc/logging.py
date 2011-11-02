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

