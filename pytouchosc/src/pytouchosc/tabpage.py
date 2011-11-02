from lxml import etree
import base64

class Tabpage(etree.ElementBase):
    """
    docstring
    """ 
    @apply
    def name():
        doc = """Tabpage Name"""
        def fget(self):
            return base64.b64decode(self.get('name'))
        def fset(self, value):
            self.set('name', base64.b64encode(str(value)))
        return property(**locals())
    
    def getMessages(self):
        x = list()
        for control in self.getchildren():
            x+= control.getMessages().items()
        return dict(x)
    
    def getReceiveDict(self):
        """
        Generate a dictionary of all possible messages that can be received from
        TouchOSC on the iPhone or iPad.  

        @rtype: dict
        @return: Dictionary of all possible received controls 
        """
        x = list()
        for control in self.getchildren():
            x += control.getReceivableMessages().items()
        return dict(x)

    def getSendDict(self):
        """
        Generate a dictionary of all possible messages that can be sent to
        TouchOSC on the iPhone or iPad.  

        @rtype: dict
        @return: Dictionary of all possible sent controls
        """
        x = list()
        for control in self.getchildren():
            x += control.getSendableMessages().items()
        return dict(x)
    
