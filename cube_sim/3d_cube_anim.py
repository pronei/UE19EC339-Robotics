#!/bin/usr/env python3

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from itertools import product, combinations

from simulator import MySolver
from constants import *


def get_center(lines):
    '''Finds the coordinates of the geometric center of the cube'''
    pass


def translate_lines(lines, coords, iter):
    '''Translate set `iter` of 12 lines relative to the starting configuration by `coords`'''
    coords = coords.reshape(3, 1)
    lines[:, :, 2*iter : 2*iter+2] += coords
    return lines


def rotate_lines(lines, euler_angles, iter):
    '''Rotate set `iter` of 12 lines relative to the starting configuration using euler angles'''
    phi, theta, psi = euler_angles
    rotationMatrix = np.array([np.cos(theta)*np.cos(psi),
                              -np.cos(phi)*np.sin(psi) + np.sin(phi)*np.sin(theta)*np.cos(psi),
                               np.sin(phi)*np.sin(psi) + np.cos(phi)*np.sin(theta)*np.cos(psi),
                               np.cos(theta)*np.sin(psi),
                               np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(theta)*np.sin(psi),
                              -np.sin(phi)*np.cos(psi) + np.cos(phi)*np.sin(theta)*np.sin(psi),
                              -np.sin(theta),
                               np.sin(phi)*np.cos(theta),
                               np.cos(phi)*np.cos(theta)]).reshape(3, 3)
    rotationMatrix = np.repeat([rotationMatrix], 12, axis=0)
    assert 2*iter <= lines.shape[2], "invalid number of iters"
    lines[:, :, [2*iter]] = rotationMatrix @ lines[:, :, [0]]
    lines[:, :, [2*iter + 1]] = rotationMatrix @ lines[:, :, [1]]
    return lines


def gen_box_lines(side_len, N, origin=np.array([0, 0, 0])):
    '''Generates the data for the sides of a cube around 
    the origin of length equal to side_len.
    Each of the 
    '''
    lines = np.empty((12, 3, 2*N + 2))
    r = [-side_len/2, side_len/2]
    i = 0
    for s, e in combinations(np.array(list(product(r, r, r))), 2):
        if np.sum(np.abs(s - e)) <= r[1] - r[0]:
            lines[i, :, 0], lines[i, :, 1] = origin + s, origin + e
            i += 1
    return lines


def update_cube(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, 2*num:2*num+2])
        line.set_3d_properties(data[2, 2*num:2*num+2])
    return lines


def run(simParams):
    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # Creating fifty sets of 12 lines and input vectors
    data = gen_box_lines(Cube.SIDE_LENGTH, N=simParams.ITERS)
    
    # generate N input vectors
    ftVec = get_input(simParams.ITERS)
    
    # Instantiate your solver here
    # solver = MySolver(simParams.TSTEP, simParams.ITERS, ftVec) 

    # data generating loop which populates data with the poses for each iteration
    # for i in range(1, simParams.ITERS):
        # update and get the current state
        # solver.next_iter()
        # state = solver.get_x()
        # data = rotate_lines(data, state[3:], i)
        # data = translate_lines(data, state[:3], i)
    
    # Get rid of this loop when your solver is ready
    for i in range(1, simParams.ITERS+1):
        data = rotate_lines(data, np.array([i*0.08]*3), i)
        data = translate_lines(data, np.array([i*0.1]*3), i)

    # NOTE: Can't pass empty arrays into 3d version of plot()
    # set the initial value of each line object to data[i, :, 0]
    lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

    # Setting the axes properties
    ax.set_xlim3d([-simParams.BOUNDS, simParams.BOUNDS])
    ax.set_xlabel('X')

    ax.set_ylim3d([-simParams.BOUNDS, simParams.BOUNDS])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-simParams.BOUNDS, simParams.BOUNDS])
    ax.set_zlabel('Z')

    ax.set_title('3D Cube Sim')

    # Creating the Animation object
    line_anim = animation.FuncAnimation(fig, update_cube, simParams.ITERS, fargs=(data, lines),
                                    interval=simParams.TSTEP*1e3, blit=False)

    plt.show()

    # find the rotation matrix between the cube and camera
    # camRotMat = MySolver.get_rot_mat(solver.get_x(), get_camera(simParams.BOUNDS))

    # change stats such that data and camRotMat are flattened, save as
    # a 1D array and delete line 122
    # stats = np.concatenate((data.ravel(), camRotMat.ravel()), axis=None)
    stats = data.flatten()
    
    return stats


if __name__ == "__main__":
    if MySolver.PROJECT:
        for seed in SEEDS:
            stats = run(SimParams(seed))
            np.savetxt("stats_" + str(seed) + ".csv", stats, fmt="%.5f", delimiter=',', encoding='ASCII')
    else:
        stats = run(SimParams(seed=1))
