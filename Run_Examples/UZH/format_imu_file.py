import cv2
import pandas as pd
import numpy as np

if __name__ == "__main__":

    imu_read_path = '/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/imu.txt'
    imu_save_path = '/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/imu_reformatted.csv'
    # now combine imu
    raw_imu = pd.read_csv(imu_read_path, delim_whitespace=True)
    raw_imu.timestamp = raw_imu.timestamp * 1e9
    #raw_imu.ang_vel_x = raw_imu.ang_vel_x * np.pi / 180
    #raw_imu.ang_vel_y = raw_imu.ang_vel_y * np.pi / 180
    #raw_imu.ang_vel_z = raw_imu.ang_vel_z * np.pi / 180
    raw_imu = raw_imu.rename(columns={"timestamp": "#timestamp [ns]", "ang_vel_x": "w_RS_S_x [rad s^-1]", "ang_vel_y": "w_RS_S_y [rad s^-1]", "ang_vel_z": "w_RS_S_z [rad s^-1]", "lin_acc_x": "a_RS_S_x [m s^-2]", "lin_acc_y": "a_RS_S_y [m s^-2]", "lin_acc_z": "a_RS_S_z [m s^-2]"})
    raw_imu = raw_imu.loc[:,
              ['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]']]
    raw_imu.to_csv(imu_save_path, header=True, index=False)
    print(f'Done reformatting imu file')
