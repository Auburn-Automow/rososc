from lxml import etree
import base64

class Control(etree.ElementBase):
    colors = ["red", "green", "blue", "yellow", "purple", "gray", "orange"]

    @apply
    def name():
        doc = """Control Name"""
        def fget(self):
            return base64.b64decode(self.get('name'))
        def fset(self, value):
            self.set('name', base64.b64encode(str(value)))
        return property(**locals())

    @apply
    def x():
        doc = """X Position"""
        def fget(self):
            return self.get('x')
        def fset(self, value):
            if type(value) is not int:
                raise TypeError("X Position must be an integer")
            self.set('x', value)
        return property(**locals())

    @apply
    def y():
        doc = """Y Position"""
        def fget(self):
            return self.get('y')
        def fset(self, value):
            if type(value) is not int:
                raise TypeError("Y Position must be an integer")
            self.set('y', value)
        return property(**locals())

    @apply
    def width():
        doc = """Control Width"""
        def fget(self):
            return self.get('w')
        def fset(self, value):
            if type(value) is not int:
                raise TypeError("Width must be an integer")
            self.set('w', value)
        return property(**locals())

    @apply
    def height():
        doc = """Control Height"""
        def fget(self):
            return self.get('h')
        def fset(self, value):
            if type(value) is not int:
                raise TypeError("Height must be an integer")
            self.set('h', value)
        return property(**locals())

    @apply
    def color():
        doc = """Control Color"""
        def fget(self):
            return self.get('color')
        def fset(self, value):
            value = value.lower()       #Check the case, to be sure.
            if not any(value == val for val in self.colors):
                raise ValueError("color must be a value from Controls.colors")
            self.set('color', value)
        return property(**locals())

    def getMessages(self):
        sendable = self.getSendableMessages()
        receivable = self.getReceivableMessages()
        sendable[self.name].update(receivable[self.name])
        return sendable

    def getSendableMessageTypes(self):
        return {self.name:{}}

    def getSendableMessages(self):
        return {self.name:{}}
    
    def getReceivableMessageTypes(self):
        position = dict()
        position['x'] = [int]
        position['y'] = [int] 
        position['z'] = [int] 
        size = dict()
        size['w'] = [int] 
        size['h'] = [int] 
        messages = {"position":position,
                    "size":size,
                    "visibility":[bool],
                    "color":self.colors}
        return {self.name:messages}
    
    def getReceivableMessages(self):
        position = dict()
        position['x'] = self.x
        position['y'] = self.y
        position['z'] = 0 
        size = dict()
        size['w'] = self.width
        size['h'] = self.height
        messages = {"position":position,
                    "size":size,
                    "visibility":True,
                    "color":self.color}
        return {self.name:messages}

   
class TextField(Control):
    @apply
    def textSize():
        doc = """Label Font Size"""
        def fget(self):
            return self.get('size')
        def fset(self, value):
            return self.set('size', value)
        return property(**locals())

    @apply
    def background():
        doc = """Background Enabled"""
        def fget(self):
            return True if self.get('background') == 'true' else False
        def fset(self, value):
            assert type(value) is bool, "Background must be boolean"
            self.set('background', str(value).lower)
        return property(**locals())

    @apply
    def outline():
        doc = """Outline Enabled"""
        def fget(self):
            return True if self.get('outline') == 'true' else False
 
        def fset(self, value):
            assert type(value) is bool, "Outline must be boolean"
            self.set('outline', str(value).lower)
        return property(**locals())

class Label(TextField):
    @apply
    def text():
        doc = """Label Text"""
        def fget(self):
            return base64.b64decode(self.get('text'))
        def fset(self, value):
            self.set('text', base64.b64encode(str(value).encode('utf-8')))
        return property(**locals())

    def getReceivableMessageTypes(self):
        msg = super(Label, self).getReceivableMessageTypes()
        msg[self.name]['text'] = [str]
        return msg

    def getReceivableMessages(self):
        msg = super(Label, self).getReceivableMessages()
        msg[self.name]['text'] = self.text
        return msg
        

class Time(TextField):
    @apply
    def seconds():
        doc = """Display seconds on time control"""
        def fget(self):
            return True if self.get('seconds') == 'true' else False
        def fset(self, value):
            assert type(value) is bool, "Seconds must be boolean"
            self.set('seconds', str(value).lower)
        return property(**locals())

class ScalableControl(Control):
    @apply
    def scalef():
        doc = """Bottom value on the control's scale"""
        def fget(self):
            return self.get('scalef')
        def fset(self, value):
            self.set('scalef', value)
        return property(**locals())

    @apply
    def scalet():
        doc = """Top value on the control's scale"""
        def fget(self):
            return self.get('scalet')
        def fset(self, value):
            self.set('scalet', value)
        return property(**locals())

    def getSendableMessages(self):
        msg = super(ScalableControl, self).getSendableMessages()
        msg[self.name]['z'] = False
        return msg

    def getSendableMessageTypes(self):
        msg = super(ScalableControl, self).getSendableMessageTypes()
        msg[self.name]['z'] = [bool]
        return msg

class Encoder(ScalableControl):
    def getSendableMessages(self):
        msg = super(Encoder, self).getSendableMessages()
        msg[self.name][None] = False
        return msg

    def getSendableMessageTypes(self):
        msg = super(Encoder, self).getSendableMessageTypes()
        msg[self.name][None] = [bool]
        return msg

class LED(ScalableControl):
    def getReceivableMessages(self):
        msg = super(LED, self).getReceivableMessages()
        msg[self.name][None] = 0.0 
        return msg

    def getReceivableMessageTypes(self):
        msg = super(LED, self).getReceivableMessageTypes()
        msg[self.name][None] = [float, int] 
        return msg

class Button(ScalableControl):
    @apply
    def local_off():
        doc = """Option to disable local feedback"""
        def fget(self):
            return True if self.get('local_off') == 'true' else False

        def fset(self, value):
            assert type(value) is bool, "local_off must be boolean"
            self.set('local_off', str(value).lower())
        return property(**locals())

    def getReceivableMessages(self):
        msg = super(Button, self).getReceivableMessages()
        msg[self.name][None] = 0.0
        return msg

    def getReceivableMessageTypes(self):
        msg = super(Button, self).getReceivableMessageTypes()
        msg[self.name][None] = [float, int]

    def getSendableMessages(self):
        msg = super(Button, self).getSendableMessages()
        msg[self.name][None] = 0.0
        return msg

    def getSendableMessageTypes(self):
        msg = super(Button, self).getSendableMessageTypes()
        msg[self.name][None] = [float, int]
        return msg

class MultiButton(Button):
    @apply
    def number_x():
        doc = """Number of buttons in the x axis of the button array"""
        def fget(self):
            return self.get('number_x')
        def fset(self, value):
            self.set('number_x', value)
        return property(**locals())

    @apply
    def number_y():
        doc = """Number of buttons in the y axis of the button array"""
        def fget(self):
            return self.get('number_y')
        def fset(self, value):
            self.set('number_y', value)
        return property(**locals())
    
    def getSendableMessages(self):
        msg = super(MultiButton, self).getSendableMessages()
        msg[self.name][None] = [0.0] * int(self.number_x) * int(self.number_y)
        msg[self.name]["dim_x"] = int(self.number_x)
        msg[self.name]["dim_y"] = int(self.number_y)
        return msg
    
    def getReceivableMessages(self):
        msg = super(MultiButton, self).getReceivableMessages()
        msg[self.name][None] = [0.0] * int(self.number_x) * int(self.number_y)
        return msg

class XYPad(ScalableControl):
    @apply
    def invertx():
        doc = """Option to invert X axis"""
        def fget(self):
            return True if self.get('invert_x') == 'true' else False

        def fset(self, value):
            assert type(value) is bool, "invert_x must be boolean"
            self.set('invert_x', str(value).lower())
        return property(**locals())

    @apply
    def inverty():
        doc = """Option to invert Y axis"""
        def fget(self):
            return True if self.get('invert_y') == 'true' else False

        def fset(self, value):
            assert type(value) is bool, "invert_y must be boolean"
            self.set('invert_y', str(value).lower())
        return property(**locals())


    def getReceivableMessages(self):
        msg = super(XYPad, self).getReceivableMessages()
        msg[self.name][None] = [0.0, 0.0]
        return msg

    def getReceivableMessageTypes(self):
        msg = super(XYPad, self).getReceivableMessageTypes()
        msg[self.name][None] = [float, int]
        return msg

    def getSendableMessages(self):
        msg = super(XYPad, self).getSendableMessages()
        msg[self.name][None] = [0.0, 0.0]
        return msg

    def getSendableMessageTypes(self):
        msg = super(XYPad, self).getSendableMessageTypes()
        msg[self.name][None] = [float, int]
        return msg


class Dial(ScalableControl):
    @apply
    def inverted():
        doc = """Option to invert X control"""
        def fget(self):
            return True if self.get('inverted') == 'true' else False

        def fset(self, value):
            assert type(value) is bool, "inverted must be boolean"
            self.set('inverted', str(value).lower())
        return property(**locals())

    @apply
    def centered():
        doc = """Option to make dial centered"""
        def fget(self):
            return True if self.get('centered') == 'true' else False

        def fset(self, value):
            assert type(value) is bool, "centered must be boolean"
            self.set('centered', str(value).lower())
        return property(**locals())

    @apply
    def response():
        doc = """Ability to set absolute or relative response"""
        def fget(self):
            return self.get('response')
        
        def fset(self, value):
            assert any(value == val for val in ['absolute', 'relative']), \
                "%s is not a valid response (absolute/relative)" % value
            self.set('response', value)
        return property(**locals())

    def getReceivableMessages(self):
        msg = super(Dial, self).getReceivableMessages()
        msg[self.name][None] = 0.0
        return msg

    def getReceivableMessageTypes(self):
        msg = super(Dial, self).getReceivableMessageTypes()
        msg[self.name][None] = [float, int]
        return msg

    def getSendableMessages(self):
        msg = super(Dial, self).getSendableMessages()
        msg[self.name][None] = 0.0
        return msg

    def getSendableMessageTypes(self):
        msg = super(Dial, self).getSendableMessageTypes()
        msg[self.name][None] = [float, int]
        return msg


class MultiDial(Dial):
    @apply
    def number():
        doc = """Number of dials"""
        def fget(self):
            return self.get('number')

        def fset(self, value):
            self.set('number', value)
        return property(**locals())
    
    def getSendableMessages(self):
        msg = super(MultiDial, self).getSendableMessages()
        msg[self.name][None] = [0.0] * int(self.number)
        msg[self.name]["number"] = int(self.number)
        return msg
    
    def getReceivableMessages(self):
        msg = super(MultiDial, self).getReceivableMessages()
        msg[self.name][None] = [0.0] * int(self.number)
        msg[self.name]["number"] = int(self.number)
        return msg

type_class_mapping = {
                "led":          LED,
                "labelv":       Label,
                "labelh":       Label,
                "push":         Button,
                "toggle":       Button,
                "xy":           XYPad,
                "faderv":       Dial,
                "faderh":       Dial,
                "rotaryv":      Dial,
                "rotaryh":      Dial,
                "encoder":      Encoder,
                "batteryv":     TextField,
                "batteryh":     TextField,
                "timev":        Time,
                "timeh":        Time,
                "multipush":    MultiButton,
                "multitoggle":  MultiButton,
                "multifaderv":  MultiDial,
                "multifaderh":  MultiDial,
                "multixy":      XYPad
                }

def test_controls():
    lookup = etree.AttributeBasedElementClassLookup('type', type_class_mapping)
    parser = etree.XMLParser(remove_blank_text=True)
    parser.setElementClassLookup(lookup)

    tree = etree.parse("/home/mjcarroll/index.xml", parser)

    root = tree.getroot()
    t = root.getchildren()[0]
    print "Receivable Messages"
    for x in t.iterchildren():
        print x.getMessages()
        
if __name__ == '__main__':
    test_controls()


