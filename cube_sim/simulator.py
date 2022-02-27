#!/bin/usr/env python3

import numpy as np

from constants import Cube
from integral import ForwardEuler, BackwardEuler, Trapezoidal


class MySolver(object):
    """
    A class to be implemented as part of the project 1.
    You're given the template of a solver (like ODE45 but a toy implementation) which has interfaces
    `next_iter`, `get_x`, `get_x_dot`. Each of these 3 interfaces have to be implemented.
    This class serves to model a solver that a simulation engine such as Gazebo might run 
    in its backend gzserver.

    Additionally static method `get_rot_mat` is supposed to be implemented which finds the
    3x3 rotation matrix between two poses.

    The cube animation in 3d_cube_anim.py consists of a simple cube to which we
    apply random forces and torques. This is modelled as 6x1 state vector [Fx, Fy, Fz, Tx, Ty, Tz].
    F_ is force in the direction of _ axis and T_ is torque about _ axis. A few seeds are set at runtime
    which generate each of these forces/torques. The task is to consider these forces/torques as
    input to a system and generate its response by modelling the object and its state.

    You may either use integrators from integral.py or plain equations of motion to solve for the
    current iteration. Refer to the unit testing in integral.py for its usage.
    """

    PROJECT = False
    
    def __init__(self, tstep, iters, forces, x=np.zeros((6, 1)), x_dot=np.zeros((6, 1))):        
        self.forces = forces
        # [x, y, z, phi, theta, psi]
        self._x = x
        # [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
        self._x_dot = x_dot
        self.tstep = tstep
        self.iters = iters
        # you may need to use this
        self.iter = 1

        # you may either use an integrator or equations to obtain x_dot and x;
        # if you're using an integrator, initialize it here in an instance variable;
        # the integrator class is vectorized so you need not initialize it for each component
        # x_dot = BackwardEuler(x_dot_init, tstep)

    def next_iter(self):
        """Update the state variables of the model, don't return anything.
        Update x and x_dot on each iteration after calculating x_dot_dot using
        F=ma either using discrete integrators (those initialized in the constructor)
        or with equations of motion.
        """
        pass

    def get_x(self):
        pass

    def get_x_dot(self):
        pass

    @staticmethod
    def get_rot_mat(x, y):
        """Find the rotation matrix between two poses x and y"""
        rotMat = np.zeros((3, 3))
        return rotMat