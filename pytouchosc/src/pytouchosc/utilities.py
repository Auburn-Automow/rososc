# -*- coding: utf-8 -*-
#
# Copyright (c) 2011-2012, Michael Carroll
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Module containing additional utility functions for use with TouchOSC layouts.
"""

from layout import Layout

__author__ = "Michael Carroll <carroll.michael@gmail.com>"

def merge_layouts(layouts):
	"""
	Given a list of layout objects, return a layout object containing the tabpages of each.

	Appends the tabpages of the second argument to those of the first.

	Caveat: Both tabpages must be of the same mode and orientation.
	"""

	if type(layouts) is not list:
		raise ValueError("Input argument layouts is not a list.")

	for layout in layouts:
		if type(layout) is not Layout:
			raise ValueError("Input list does not contain layouts")

	mode = layouts[0].mode
	orientation = layouts[0].orientation

	for layout in layouts:
		if layout.mode != mode:
			raise ValueError("Input list contains layouts with different modes")
		if layout.orientation != orientation:
			raise ValueError("Input list contains layouts with different orientations")

