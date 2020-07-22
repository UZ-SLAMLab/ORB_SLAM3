import numpy as np
import csv
import re
import argparse
import sys

comment_pattern = re.compile(r'\s*#.*$')


class Position:
    def __init__(self, timestamp, pos=None, rot_quat=None):
        self.timestamp = timestamp
        self.is_kf = False
        if pos is None and rot_quat is None:
            self.exist = False
            self.pos = pos  # x,y,z
            self.rot_quat = rot_quat  # w,x,y,z
        else:
            self.exist = True
            self.pos = pos  # x,y,z
            self.rot_quat = rot_quat  # w,x,y,z


class Trajectory:
    def __init__(self, lst_poses=None):
        if lst_poses is not None:
            self.num_poses = len(lst_poses)
            self.timestamps = np.zeros((self.num_poses, 1))
            self.pos = np.zeros((self.num_poses, 3))
            self.q = np.zeros((self.num_poses, 4))
            for i in range(0, self.num_poses):
                self.timestamps[i] = lst_poses[i].timestamp
                if lst_poses[i].rot_quat is not None:
                    for j in range(0, 4):
                        self.q[i][j] = lst_poses[i].rot_quat[j]
                for j in range(0, 3):
                    self.pos[i][j] = lst_poses[i].pos[j]
            # self.pos = np.transpose(self.pos)
            self.pos = np.transpose(self.pos)
            self.q = np.transpose(self.q)


def load_file(path_file_name: str, delimiter=" "):
    lst_poses = []
    with open(path_file_name) as file:
        file = skip_comments(file)
        for line in file:
            vector = line.split(delimiter)
            # print(len(vector))
            if len(vector) == 8:
                # Correct position

                timestamp = float(vector[0])
                pos = [float(vector[1]), float(vector[2]), float(vector[3])]
                rot_quat = [float(vector[7]), float(vector[4]), float(vector[5]), float(vector[6])]
                pose = Position(timestamp, pos, rot_quat)
                lst_poses.append(pose)
            else:
                # We haven't got a position in this
                timestamp = float(vector[0])
                lst_poses.append(Position(timestamp))

    return lst_poses


def associate(poses_test, poses_gt):
    num_match = 0
    i_test = 0
    i_gt = 0
    lst_check_test = []
    lst_check_gt = []
    while i_test < len(poses_test):
        if i_gt == len(poses_gt):
            break
            lst_check_test.append(poses_test[i_test])
            lst_check_gt.append(poses_gt[i_gt-1])
            num_match = num_match + 1
            i_test = i_test + 1
        elif np.abs(poses_test[i_test].timestamp - poses_gt[i_gt].timestamp)/1e3 < 1:
            lst_check_test.append(poses_test[i_test])
            lst_check_gt.append(poses_gt[i_gt])
            num_match = num_match + 1
            i_test = i_test + 1
            i_gt = i_gt + 1
        elif poses_test[i_test].timestamp < poses_gt[i_gt].timestamp:
            i_test = i_test + 1
            if i_test >= len(poses_test):
                break
        elif poses_test[i_test].timestamp > poses_gt[i_gt].timestamp:
            i_gt = i_gt + 1
            if i_gt >= len(poses_gt):
                break

    return lst_check_test, lst_check_gt


def create_trajectory(lst_poses_test, lst_poses_gt):
    lst_exist_poses_test = []
    lst_exist_poses_gt = []
    for i in range(len(lst_poses_test)):
        if lst_poses_test[i].exist:
            lst_exist_poses_test.append(lst_poses_test[i])
            pose_gt = lst_poses_gt[i]

            lst_exist_poses_gt.append(pose_gt)

    traj_test = Trajectory(lst_exist_poses_test)
    traj_gt = Trajectory(lst_exist_poses_gt)
    return traj_test, traj_gt


def skip_comments(lines):
    """
    A filter which skip/strip the comments and yield the
    rest of the lines

    :param lines: any object which we can iterate through such as a file
        object, list, tuple, or generator
    """
    global comment_pattern

    for line in lines:
        line = re.sub(comment_pattern, '', line).strip()
        if line:
            yield line


def align(traj_test, traj_gt):
    mean_test = traj_test.pos.mean(1)
    mean_test = mean_test.reshape(mean_test.size, 1)
    mean_gt = traj_gt.pos.mean(1)
    mean_gt = mean_gt.reshape(mean_gt.size, 1)

    traj_centered_test = Trajectory()
    traj_centered_gt = Trajectory()
    traj_centered_test.pos = traj_test.pos - mean_test
    traj_centered_gt.pos = traj_gt.pos - mean_gt

    L = traj_centered_test.pos.shape[1]

    W = np.zeros((3, 3))
    for col in range(traj_centered_test.pos.shape[1]):
        W += np.outer(traj_centered_test.pos[:, col], traj_centered_gt.pos[:, col])
    U, d, Vh = np.linalg.linalg.svd(W.transpose())

    S = np.matrix(np.identity(3))
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        # print("Negative")
        S[2, 2] = -1

    Rot = U * S * Vh

    # pose_rot_cent_test = np.matmul(Rot, traj_centered_test.pos)
    pose_rot_cent_test = Rot * np.matrix(traj_centered_test.pos)
    # print('Size center: {}'.format(pose_rot_cent_test.shape))
    # print('Size center2: {}'.format((Rot * np.matrix(traj_centered_test.pos)).shape))

    s = 1.0
    dots = 0.0
    norms = 0.0
    for column in range(traj_centered_gt.pos.shape[1]):
        dots += np.dot(traj_centered_gt.pos[:, column].transpose(), pose_rot_cent_test[:, column])
        normi = np.linalg.norm(traj_centered_test.pos[:, column])
        norms += normi * normi

    s = float(dots / norms)

    traj_length = 0
    for i in range(1, traj_centered_gt.pos.shape[1]):
        # print("Pos = ", _trajData.pos[:, i])
        traj_length = traj_length + np.linalg.norm(traj_centered_gt.pos[:, i] - traj_centered_gt.pos[:, i - 1])

    # print("Rotation matrix between model and GT: \n", Rot)

    trans_scale = mean_gt - s * np.matmul(Rot, mean_test)
    traj_aligned_scale = Trajectory()
    traj_aligned_scale.pos = s * Rot * np.matrix(traj_test.pos) + trans_scale

    trans = mean_gt - np.matmul(Rot, mean_test)
    traj_aligned = Trajectory()
    traj_aligned.pos = Rot * np.matrix(traj_test.pos) + trans

    # np.reshape(_traj_test.pos, (3, L))
    # print(_traj_test.pos.shape)
    error_traj_scale = traj_aligned_scale.pos - traj_gt.pos
    error_traj = traj_aligned.pos - traj_gt.pos

    ate_scale = np.squeeze(np.asarray(np.sqrt(np.sum(np.multiply(error_traj_scale, error_traj_scale), 0))))
    ateRMSE_scale = np.sqrt(np.dot(ate_scale, ate_scale) / len(ate_scale))

    ate = np.squeeze(np.asarray(np.sqrt(np.sum(np.multiply(error_traj, error_traj), 0))))
    ateRMSE = np.sqrt(np.dot(ate, ate) / len(ate))

    # Print the error
    print("ATE RMSE(7DoF): " + str(ateRMSE_scale))
    print("scale: %f " % s)
    print("ATE RMSE(6DoF): " + str(ateRMSE))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
        This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
        ''')

    parser.add_argument('SLAM', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('GT', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    args = parser.parse_args()

    str_file_gt = args.GT
    str_file_traj = args.SLAM

    lst_gt_poses = load_file(str_file_gt, ",")
    lst_traj_poses = load_file(str_file_traj, " ")

    lst_traj_poses2, lst_gt_poses2 = associate(lst_traj_poses, lst_gt_poses)
    traj_test, traj_gt = create_trajectory(lst_traj_poses2, lst_gt_poses2)

    align(traj_test, traj_gt)

