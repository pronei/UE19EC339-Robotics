#! /usr/bin/env python3

import math

def myAtan2(x, y):
    return math.atan2(y, -x)

# TODO
def iKine(point):
    '''Solve for the inverse kinematics of the RPP arm.
    Return a list of 3 elements containing [theta, vertical_distance, horizontal_distance].
    Please use myAtan2 defined above instead of math.atan2
    '''
    return [0] * 3
