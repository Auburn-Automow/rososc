#!/usr/bin/env python

PKG = 'pytouchosc'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

modules_to_test = ['test_layout', 'test_utilities', 'test_layoutserver']

if __name__ == '__main__':
    import rosunit

    for module in modules_to_test:
        try:
            exec("import " + module + " as ModuleUnderTest")
        except Exception as e:
            print e
            raise Exception("Unloadable module " + module)
        
        (_,_,module_name) = module.partition('_')

        tests = ModuleUnderTest.rostest()
        for (name, test) in tests:
            coverage = PKG + '.' + module_name
            rosunit.unitrun(PKG, name, test, coverage_packages=coverage)
