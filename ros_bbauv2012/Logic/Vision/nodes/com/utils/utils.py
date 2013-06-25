#!/usr/bin/env python2
'''
Utilities common to the vision tasks
'''

'''
Normalizes heading of the machine.
'''
def norm_heading(heading):
    if heading >= 360:
        return heading - 360
    if heading < 0:
        return heading + 360
    return heading

'''
Converts angles from calculated image space to heading space.
'''
def to_heading_space(angle):
    return angle + 90

'''
Clamps values to between minimum and maximum.
'''
def clamp(val, minimum, maximum):
    return max(minimum, min(val, maximum))

