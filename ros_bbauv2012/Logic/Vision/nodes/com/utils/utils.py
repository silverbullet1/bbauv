#!/usr/bin/env python2
'''
Utilities common to the vision tasks
'''

# Helper function to normalize heading
def norm_heading(heading):
    if heading >= 360:
        return heading - 360
    if heading < 0:
        return heading + 360
    return heading

# Helper function to convert angles from calculated image space to heading space
def to_heading_space(angle):
    return angle + 90

