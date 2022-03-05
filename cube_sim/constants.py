#!/usr/bin/env python3
# encoding: utf-8

import numpy as np

"""
Constants for the simulation and cube such as mass, moments of inertia.
Simulation constants include seed, timestep and # of iterations.
Also includes the poses of the cameras in the global frame of reference.
Everthing is in SI units and orientations are described using Euler angles.
"""

SEEDS = [2, 5, 7, 11, 13]

# Simulation parameters
class SimParams(object):
    TSTEP = 5e-2
    ITERS = 50
    BOUNDS = 5.0

    def __init__(self, seed):
        np.random.seed(seed)
        self.seed = seed


# Physical dimensions of the cube in SI units
class Cube(object):
    SIDE_LENGTH = 2
    MASS = 1
    MOMENT_OF_INERTIA = [0.66667]*3


# generate the pose of a camera within bounds
def get_camera(bounds):
    pose = np.zeros((6, 1))
    pose[:3] = np.random.randint(bounds+1, size=(3, 1))
    pose[3:] = np.random.uniform(0, 2*np.pi, size=(3, 1))
    return pose

# generate N random input vectors of [F_vec, T_vec]
def get_input(N):
    return np.random.random(size=(N, 6, 1))
