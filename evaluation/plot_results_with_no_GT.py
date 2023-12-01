# Modified by Raul Mur-Artal
# Automatically compute the optimal scale factor for monocular VO/SLAM.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy as np
import associate
import pandas
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as scipy_rotation

matplotlib.use('TkAgg')


def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    """

    np.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity(3))
    if (np.linalg.det(U) * np.linalg.det(Vh) < 0):
        S[2, 2] = -1
    rot = U * S * Vh

    rotmodel = rot * model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:, column].transpose(), rotmodel[:, column])
        normi = np.linalg.norm(model_zerocentered[:, column])
        norms += normi * normi

    s = float(dots / norms)
    trans = data.mean(1) - s * rot * model.mean(1)
    model_aligned = s * rot * model + trans
    alignment_error = model_aligned - data
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), 0)).A[0]

    return rot, trans, trans_error, s


def plot_traj(ax, stamps, traj, style, color, label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = np.median([s - t for s, t in zip(stamps[1:], stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i] - last < 2 * interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x) > 0:
            ax.plot(x, y, z, style, color=color, label=label)
            label = ""
            x = []
            y = []
            z = []
        last = stamps[i]
    if len(x) > 0:
        ax.plot(x, y, z, style, color=color, label=label)


if __name__ == "__main__":
    # Dataset path
    orb_results_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/ORBSLAM_Results/monocular_frames.txt'
    evaluation_results_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/ORBSLAM_Results/Monocular'
    quaternion_order = [4, 5, 6, 3]
    # save evaluation output
    plot_results_output_file = evaluation_results_path + '_trajectory_output_frames.png'
    plot_results_output_file_2d = evaluation_results_path + '_trajectory_output_frames_2d.png'
    plot_results_output_file_orientation = evaluation_results_path + '_trajectory_output_frames_orientation.png'

    results_list = associate.read_file_list(orb_results_path, False)
    time_stamp = list(results_list.keys())
    time_stamp = (np.array(time_stamp) - time_stamp[0]) * 1e-9
    results_xyz = np.array([[float(value) for value in results_list[key][0:3]] for key in list(results_list.keys())])
    results_orientation_quaternions = np.array([[float(value) for value in results_list[key][quaternion_order]] for key in list(results_list.keys())])
    results_orientation_euler = np.array([scipy_rotation.from_quat(results_orientation_quaternions[i]).as_euler('zyx', degrees=True).flatten() for i in range(len(results_orientation_quaternions))])



    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(time_stamp, np.array(results_xyz[:, 0]).flatten(), c="black", label="estimation x")
    ax.plot(time_stamp, np.array(results_xyz[:, 1]).flatten(), c="black", label="estimation y")
    ax.plot(time_stamp, np.array(results_xyz[:, 2]).flatten(), c="black", label="estimation z")
    ax.legend()
    ax.set_xlabel('time [sec]')
    ax.set_ylabel('x, y, z [m]')
    plt.savefig(plot_results_output_file_2d, format="png")
    plt.close()
    plt.clf()
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(time_stamp, np.array(results_orientation_euler[:, 0]).flatten(), c="black", label="estimation euler x")
    ax.plot(time_stamp, np.array(results_orientation_euler[:, 1]).flatten(), c="black", label="estimation euler y")
    ax.plot(time_stamp, np.array(results_orientation_euler[:, 2]).flatten(), c="black", label="estimation euler z")
    ax.legend()
    ax.set_xlabel('time [sec]')
    ax.set_ylabel('euler x, y, z [rad]')
    plt.savefig(plot_results_output_file_orientation, format="png")
    plt.close()
    plt.clf()
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    plot_traj(ax, time_stamp, results_xyz, '-', "blue", "estimated")
    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.savefig(plot_results_output_file, format="png")
    plt.close()
    plt.clf()
