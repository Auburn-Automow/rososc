import roslib; roslib.load_manifest('touchosc_bridge')

import pytouchosc.controls

import copy
from lxml import etree


class AggregateControl(object):
    def __init__(self, name, position_x = 0, position_y = 0):
        self._name = name
        self._x = position_x
        self._y = position_y

        self.controls = {}

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = int(value)

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = int(value)

    @property
    def width(self):
        if len(self.controls):
            widths = [int(c.width) + int(c.x) for c in self.controls.itervalues()]
            return max(widths)
        else:
            return 0

    @property
    def height(self):
        if len(self.controls):
            heights = [int(c.height) + int(c.y) for c in self.controls.itervalues()]
            return max(heights)
        else:
            return 0

    def add_control(self, control):
        """
        Add a control to the aggregate control instance.  Returns the current 
        number of controls in the instance.
        """
        self.controls[control.name] = control
        return len(self.controls)

    def to_xml(self, pretty_print = False):
        return "\n".join([etree.tostring(c, pretty_print=pretty_print) for 
                         c in self.controls.itervalues()])

    def add_to_tabpage(self, tabpage):
        for c in self.controls.itervalues():
            tabpage.append(c)

class DisplayList(AggregateControl):
    def __init__(self, name, length, height, width):
        """
        Constructor for DisplayList
        
        @type name: string
        @param name: Name of DisplayList object, prefixed to all control names.
        @type length: int
        @param length: Number of items in the display list
        @type height: int
        @param height: Display height of the display list
        @type width: int
        @param width: Display width of the display list
        """
        if (height % length) != 0:
            raise ValueError("Width must be evenly divisible by length")

        super(DisplayList, self).__init__(name)
        self._width = width
        self._length = length
        self._height = height
        self._create_labels()

    def _create_labels(self):
        """
        Create the label objects and add them to the instance
        """
        # Already guaranteed to be safe by the constructor
        stride = int(self._height / self._length)
        for i in range(self._length):
            control_name = self.name + "-label%d"%i
            c = pytouchosc.controls.control_factory("labelh", control_name)
            c.background = True
            c.outline = False
            c.textSize = 14
            c.y = i * stride
            c.height = stride
            c.width = self._width
            c.text = ""
            self.controls[control_name] = c

        c = pytouchosc.controls.control_factory("multitoggle", self.name)
        c.number_x = 1
        c.number_y = self._length
        c.ex_mode = "true"
        c.height = self._height
        c.width = self.width
        self.controls[self.name] = c


