#!/bin/usr/env python3

import numpy as np

from constants import Cube
from integral import ForwardEuler, BackwardEuler, Trapezoidal


class MySolver(object):
    """
    A class to be implemented as part of the project I.
    This class serves to model a solver that a simulation engine such as Gazebo might run 
    in its backend gzserver.

    You're given the template of a solver (like ODE45 but a toy implementation) which has an
    interface `next_iter`. This interface, which is meant to to calculate and update the state,
    has to be implemented.
    
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

    # TODO: Change this, there won't be additional todos in this project, the implementation
    # is completely left up to your liking :)
    PROJECT = False
    
    def __init__(self, tstep, iters, forces, x=np.zeros((6, 1)), x_dot=np.zeros((6, 1))):        
        # Access this array for force applied in each iteration
        self.forces = forces
        # [x, y, z, phi, theta, psi]
        self._x = x
        # [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]
        self._x_dot = x_dot
        self.tstep = tstep
        self.iters = iters
        # you may need to use this and update in `next_iter`
        self.iter = 0

        # you may either use an integrator or equations to obtain x_dot and x;
        # if you're using an integrator, initialize it here in an instance variable;
        # the integrator class is vectorized so you need not initialize it for each component
        # integral_x_dot = BackwardEuler(x_init, tstep)

    def next_iter(self):
        """Update the state variables of the model, don't return anything.
        Update x and x_dot on each iteration. Calculate x_dot_dot using
        Newton's laws of motion. Use either discrete integrators or
        equations of motion to find x_dot and x.
        """
        pass

    def get_x(self):
        return self._x

    def set_x(self, new_x):
        self._x = new_x

    def get_x_dot(self):
        return self._x_dot

    def set_x_dot(self, new_x_dot):
        self._x_dot = new_x_dot

    @staticmethod
    def get_rot_mat(x, y):
        """Find the homogenous rotation matrix between two poses x and y"""
        homoRotMat = np.zeros((4, 4))
        rotMat = np.zeros((3, 3))
        translationVec = np.zeros((3, 1))
        # Fill in `rotMat` and `translationVec` here
        homoRotMat[:3, :3] = rotMat
        homoRotMat[:-1, [-1]] = translationVec
        return homoRotMat
