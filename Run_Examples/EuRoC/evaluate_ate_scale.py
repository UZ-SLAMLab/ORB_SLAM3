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
import evaluation.associate as associate
import pandas
import matplotlib
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as scipy_rotation
from mpl_toolkits.mplot3d import Axes3D

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
    # print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)
    is_verbose = False
    plot_orientation_error_euler_angles = False
    # save all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)
    is_save_verbose = True
    # save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)
    is_save_associations = True
    # save aligned second trajectory to disk (format: stamp2 x2 y2 z2)
    is_save = True
    # plot the first and the aligned second trajectory to an image (format: png)
    is_plot = True
    # maximally allowed time difference for matching entries (default: 10000000 ns)
    max_time_difference_ns = 10000000
    # time offset added to the timestamps of the second file (default: 0.0)
    time_offset = 0
    # scaling factor for the second trajectory (default: 1.0)
    scale_factor = 1.0
    # Dataset path
    dataset_path = '/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/'
    results_path = 'ORBSLAM3_Run/'
    orb_results_path = '/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH01/results_frames.txt'
    ground_truth_path = '/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/MH01/mav0/state_groundtruth_estimate0/data.csv'
    evaluation_results_path = '/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH01'
    # Evaluation Job
    first_file = ground_truth_path
    second_file = orb_results_path
    # create the output directory if it doesn't exist
    if not os.path.exists(evaluation_results_path):
        os.makedirs(evaluation_results_path)
        print('new directory is created for output')
    # save evaluation output
    poses_associations_output_file = evaluation_results_path + '/poses_associations_frames.txt'
    time_associations_output_file = evaluation_results_path + '/time_associations_frames.txt'
    estimation_error_summary_output_file = evaluation_results_path + '/evaluation_error_frames.txt'
    plot_results_output_file = evaluation_results_path + '/trajectory_output_frames.png'
    plot_error_hist_output_file = evaluation_results_path + '/error_histogram_frames.png'
    plot_error_rot_hist_output_file = evaluation_results_path + '/error_histogram_rotation_frames.png'

    first_list = associate.read_file_list(first_file, False)
    second_list = associate.read_file_list(second_file, False)

    matches = associate.associate(first_list, second_list, float(time_offset), float(max_time_difference_ns))
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a, b in matches]).transpose()
    second_xyz = np.matrix(
        [[float(value) * float(scale_factor) for value in second_list[b][0:3]] for a, b in matches]).transpose()
    dictionary_items = second_list.items()
    sorted_second_list = sorted(dictionary_items)

    second_xyz_full = np.matrix(
        [[float(value) * float(scale_factor) for value in sorted_second_list[i][1][0:3]] for i in
         range(len(sorted_second_list))]).transpose()  # sorted_second_list.keys()]).transpose()
    rot, trans, trans_error, scale = align(second_xyz, first_xyz)

    second_xyz_aligned = scale * rot * second_xyz + trans
    first_stamps = list(first_list.keys())
    first_stamps.sort()
    first_xyz_full = np.matrix(
        [[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()

    second_stamps = list(second_list.keys())
    second_stamps.sort()
    second_xyz_full = np.matrix(
        [[float(value) * float(scale_factor) for value in second_list[b][0:3]] for b in
         second_stamps]).transpose()
    second_xyz_full_aligned = scale * rot * second_xyz_full + trans

    # Orientation
    quaternion_order_1 = [3, 4, 5, 6]
    quaternion_order_2 = [4, 5, 6, 3]
    first_orientation_matrices = np.array([scipy_rotation.from_quat(first_list[a][quaternion_order_2]).as_matrix() for a, _ in matches])
    second_orientation_matrices = np.array([scipy_rotation.from_quat(second_list[b][quaternion_order_1]).as_matrix()  for _, b in matches])
    second_orientation_matrices_aligned = np.array([rot * second_orientation_matrices[i] for i in range(len(second_orientation_matrices))])
    orientation_difference_matrices = np.array([np.matmul(np.transpose(first_orientation_matrices[i]), second_orientation_matrices_aligned[i]) for i in range(len(second_orientation_matrices_aligned))])
    reference_rotation_difference_matrix = np.mean(orientation_difference_matrices, axis=0)
    second_orientation_matrices_aligned = np.array([rot * second_orientation_matrices[i] * np.transpose(reference_rotation_difference_matrix) for i in range(len(second_orientation_matrices))])
    orientation_error_matrices = np.array([np.matmul(np.transpose(first_orientation_matrices[i]), second_orientation_matrices_aligned[i]) for i in range(len(second_orientation_matrices_aligned))])
    orientation_error_euler = np.array([scipy_rotation.from_matrix(orientation_error_matrices[i]).as_euler('zyx', degrees= True) for i in range(len(first_orientation_matrices))])
    orientation_error_axis_angle = np.array([scipy_rotation.from_matrix(orientation_error_matrices[i]).as_rotvec(degrees=True) for i in range(len(first_orientation_matrices))])
    orientation_diff = np.array([np.matmul(np.matmul(np.transpose(rot), first_orientation_matrices[i]), np.transpose(second_orientation_matrices[i])) for i in range(len(second_orientation_matrices_aligned))])
    euler_rmse = np.sqrt(np.mean(orientation_error_euler**2, axis=0))
    angle_error = np.array([np.linalg.norm(orientation_error_axis_angle[i]) for i in range(len(first_orientation_matrices))])
    angle_rmse = np.sqrt(np.mean(angle_error**2))

    if plot_orientation_error_euler_angles:
        first_orientation_euler_y = np.array([scipy_rotation.from_matrix(first_orientation_matrices[i]).as_euler('zyx', degrees=True)[2] for i in range(len(first_orientation_matrices))])
        second_orientation_euler_y = np.array([scipy_rotation.from_matrix(second_orientation_matrices[i]).as_euler('zyx', degrees=True)[2] for i in range(len(second_orientation_matrices_aligned))])
        second_orientation_euler_y_aligned = np.array([scipy_rotation.from_matrix(second_orientation_matrices_aligned[i]).as_euler('zyx', degrees=True)[2] for i in range(len(second_orientation_matrices_aligned))])
        orientation_error_euler_y = np.array([scipy_rotation.from_matrix(orientation_error_matrices[i]).as_euler('zyx', degrees=True)[2] for i in range(len(orientation_error_matrices))])
        plt.plot(first_orientation_euler_y, label='first')
        plt.plot(second_orientation_euler_y, label='second')
        plt.plot(second_orientation_euler_y_aligned, label='second aligned')
        plt.plot(orientation_error_euler_y, label='error')
        plt.legend()
        plt.show()

    if is_verbose:
        print(f"compared_pose_pairs %d pairs" % (len(trans_error)))
        print(f"absolute_translational_error.rmse %f m" % np.sqrt(
            np.dot(trans_error, trans_error) / len(trans_error)))
        print(f"absolute_translational_error.mean %f m" % np.mean(trans_error))
        print(f"absolute_translational_error.median %f m" % np.median(trans_error))
        print(f"absolute_translational_error.std %f m" % np.std(trans_error))
        print(f"absolute_translational_error.min %f m" % np.min(trans_error))
        print(f"absolute_translational_error.max %f m" % np.max(trans_error))
        print(f"absolute_rotational_angle_degrees_error.rmse %f m" % angle_rmse)
        print(f"max idx: %i" % np.argmax(trans_error))

    if is_save_verbose:
        headers = ["# pose paired compared", "rmse of absolute_translational_error",
                   "rmse of orientation error rotational angle in axis-angle",
                   "mean of absolute_translational_error", "median of absolute_translational_error",
                   "std of absolute_translational_error", "min of absolute_translational_error",
                   "max of absolute_translational_error"]
        output_df = pandas.DataFrame(
            [len(trans_error), np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)), angle_rmse,
             np.mean(trans_error), np.median(trans_error), np.std(trans_error), np.min(trans_error),
             np.max(trans_error)])
        output_df.index = headers
        output_df.to_csv(estimation_error_summary_output_file)

    if is_save_associations:
        file = open(poses_associations_output_file, "w")
        file.write("\n".join(
            ["%f %f %f %f %f %f %f %f" % (a, x1, y1, z1, b, x2, y2, z2) for (a, b), (x1, y1, z1), (x2, y2, z2)
             in zip(matches, first_xyz.transpose().A, second_xyz_aligned.transpose().A)]))
        file.close()

    if is_save:
        file = open(time_associations_output_file, "w")
        file.write("\n".join(["%f " % stamp + " ".join(["%f" % d for d in line]) for stamp, line in
                              zip(second_stamps, second_xyz_aligned.transpose().A)]))
        file.close()

    if is_plot:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        plot_traj(ax, first_stamps, first_xyz_full.transpose().A, '-', "black", "ground truth")
        plot_traj(ax, second_stamps, second_xyz_full_aligned.transpose().A, '-', "blue", "estimated")
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(plot_results_output_file, format="png")
        plt.close()
        position_error = first_xyz - second_xyz_aligned
        plt.clf()
        fig, ax = plt.subplots(3)
        fig.suptitle('Translational Error Histograms')
        ax[0].hist(np.array(position_error[0, :]).flatten(), label='error in x', bins=100, color='blue')
        ax[1].hist(np.array(position_error[1, :]).flatten(), label='error in y', bins=100, color='green')
        ax[2].hist(np.array(position_error[2, :]).flatten(), label='error in z', bins=100, color='red')
        ax[2].set_xlabel('error [m]')
        fig.legend()
        plt.savefig(plot_error_hist_output_file, format="png")
        plt.close()
        plt.clf()
        fig, ax = plt.subplots(3)
        fig.suptitle('Rotational Error Histograms')
        ax[0].hist(np.array(orientation_error_euler[:, 0]).flatten(), label='error in euler x', bins=100, color='blue')
        ax[1].hist(np.array(orientation_error_euler[:, 1]).flatten(), label='error in euler y', bins=100, color='green')
        ax[2].hist(np.array(orientation_error_euler[:, 2]).flatten(), label='error in euler z', bins=100, color='red')
        ax[2].set_xlabel('error [degrees]')
        fig.legend()
        plt.savefig(plot_error_rot_hist_output_file, format="png")
        plt.close()
