import cv2
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from ahrs.filters import Mahony
from scipy.spatial.transform import Rotation

if __name__ == "__main__":

    imu_read_path = '/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/imu.txt'
    imu_save_path = '/home/justmohsen/Documents/SLAM/Datasets/UZH/indoor_forward_3_snapdragon/imu_reformatted.csv'
    # now combine imu
    raw_imu = pd.read_csv(imu_read_path, delim_whitespace=True)
    raw_imu.timestamp = raw_imu.timestamp * 1e9
    #raw_imu.ang_vel_x = raw_imu.ang_vel_x * np.pi / 180
    #raw_imu.ang_vel_y = raw_imu.ang_vel_y * np.pi / 180
    #raw_imu.ang_vel_z = raw_imu.ang_vel_z * np.pi / 180

    raw_imu.ang_vel_x = raw_imu.ang_vel_x - np.mean(raw_imu.ang_vel_x[0:4000])
    raw_imu.ang_vel_y = raw_imu.ang_vel_y - np.mean(raw_imu.ang_vel_y[0:4000])
    raw_imu.ang_vel_z = raw_imu.ang_vel_z - np.mean(raw_imu.ang_vel_z[0:4000])

    raw_imu = raw_imu.rename(columns={"timestamp": "#timestamp [ns]", "ang_vel_x": "w_RS_S_x [rad s^-1]", "ang_vel_y": "w_RS_S_y [rad s^-1]", "ang_vel_z": "w_RS_S_z [rad s^-1]", "lin_acc_x": "a_RS_S_x [m s^-2]", "lin_acc_y": "a_RS_S_y [m s^-2]", "lin_acc_z": "a_RS_S_z [m s^-2]"})

    raw_imu = raw_imu.loc[:,
              ['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]']]
    raw_imu.to_csv(imu_save_path, header=True, index=False)
    print(f'Done reformatting imu file')

    raw_imu = pd.read_csv(imu_read_path, delim_whitespace=True)
    imu_euroc_read_path = '/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/MH01/mav0/imu0/data.csv'
    raw_imu_euroc = pd.read_csv(imu_euroc_read_path)
    acc_x_euroc = raw_imu_euroc.loc[:,'a_RS_S_x [m s^-2]']
    acc_y_euroc = raw_imu_euroc.loc[:,'a_RS_S_y [m s^-2]']
    acc_z_euroc = raw_imu_euroc.loc[:,'a_RS_S_z [m s^-2]']
    acc_x_uzh = raw_imu.lin_acc_x
    acc_y_uzh = raw_imu.lin_acc_y
    acc_z_uzh = raw_imu.lin_acc_z
    w_x_euroc = raw_imu_euroc.loc[:,'w_RS_S_x [rad s^-1]']
    w_y_euroc = raw_imu_euroc.loc[:,'w_RS_S_y [rad s^-1]']
    w_z_euroc = raw_imu_euroc.loc[:,'w_RS_S_z [rad s^-1]']
    w_x_uzh = raw_imu.ang_vel_x
    w_y_uzh = raw_imu.ang_vel_y
    w_z_uzh = raw_imu.ang_vel_z

    w_x_uzh = w_x_uzh - np.mean(w_x_uzh[0:4000])
    w_y_uzh = w_y_uzh - np.mean(w_y_uzh[0:4000])
    w_z_uzh = w_z_uzh - np.mean(w_z_uzh[0:4000])


    print(np.mean(np.abs(acc_x_euroc)))
    print(np.mean(np.abs(acc_y_euroc)))
    print(np.mean(np.abs(acc_z_euroc)))
    print(np.mean(np.abs(acc_x_uzh)))
    print(np.mean(np.abs(acc_y_uzh)))
    print(np.mean(np.abs(acc_z_uzh)))

    print(np.mean(np.abs(w_x_euroc)))
    print(np.mean(np.abs(w_y_euroc)))
    print(np.mean(np.abs(w_z_euroc)))
    print(np.mean(np.abs(w_x_uzh)))
    print(np.mean(np.abs(w_y_uzh)))
    print(np.mean(np.abs(w_z_uzh)))

    print(np.max(np.abs(acc_x_euroc)))
    print(np.max(np.abs(acc_y_euroc)))
    print(np.max(np.abs(acc_z_euroc)))
    print(np.max(np.abs(acc_x_uzh)))
    print(np.max(np.abs(acc_y_uzh)))
    print(np.max(np.abs(acc_z_uzh)))

    print(np.max(np.abs(w_x_euroc)))
    print(np.max(np.abs(w_y_euroc)))
    print(np.max(np.abs(w_z_euroc)))
    print(np.max(np.abs(w_x_uzh)))
    print(np.max(np.abs(w_y_uzh)))
    print(np.max(np.abs(w_z_uzh)))

    frames = list(range(1, 100))
    #frames = list(range(1, len(acc_x_euroc)))
    plt.plot(acc_x_euroc[frames], label='EuRoC Acc X')
    plt.plot(acc_y_euroc[frames], label='EuRoC Acc Y')
    plt.plot(acc_y_euroc[frames], label='EuRoC Acc Z')
    plt.plot(acc_x_uzh[frames], label='UZH Acc X')
    plt.plot(acc_y_uzh[frames], label='UZH Acc Y')
    plt.plot(acc_z_uzh[frames], label='UZH Acc Z')
    plt.legend()
    plt.title('Acceleration Values for Euroc vs UZH datasets')
    plt.show()

    frames = list(range(20000, 20100))
    frames = list(range(1, len(acc_x_euroc)))
    plt.plot(w_x_euroc[frames], label='EuRoC w X')
    plt.plot(w_y_euroc[frames], label='EuRoC w Y')
    plt.plot(w_y_euroc[frames], label='EuRoC w Z')
    plt.plot(w_x_uzh[frames], label='UZH w X')
    plt.plot(w_y_uzh[frames], label='UZH w Y')
    plt.plot(w_z_uzh[frames], label='UZH w Z')
    plt.legend()
    plt.title('Angular Velocity Values for Euroc vs UZH datasets')
    plt.show()

    orientation_euroc = Mahony(gyr=np.array([w_x_euroc, w_y_euroc, w_z_euroc]).transpose(), acc=np.array([acc_x_euroc, acc_y_euroc, acc_z_euroc]).transpose(), frequency=200.0)
    orientation_uzh = Mahony(gyr=np.array([w_x_uzh, w_y_uzh, w_z_uzh]).transpose(), acc=np.array([acc_x_uzh, acc_y_uzh, acc_z_uzh]).transpose(), frequency=500.0)

    #frames = list(range(20000, 21000))
    frames = list(range(1, len(acc_x_euroc)))
    plt.plot(orientation_euroc.Q[frames,0], label='EuRoC q X')
    plt.plot(orientation_euroc.Q[frames,1], label='EuRoC q Y')
    plt.plot(orientation_euroc.Q[frames,2], label='EuRoC q Z')
    plt.plot(orientation_euroc.Q[frames,3], label='EuRoC q W')
    plt.plot(orientation_uzh.Q[frames,0], label='UZH q X')
    plt.plot(orientation_uzh.Q[frames,1], label='UZH q Y')
    plt.plot(orientation_uzh.Q[frames,2], label='UZH q Z')
    plt.plot(orientation_uzh.Q[frames,3], label='UZH q W')
    plt.legend()
    plt.title('Quaternions Values for Euroc vs UZH datasets')
    plt.show()

    rot = Rotation.from_quat(orientation_euroc.Q)
    rot_euler = rot.as_euler('xyz', degrees=True)
    euler_euroc = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])
    rot = Rotation.from_quat(orientation_uzh.Q)
    rot_euler = rot.as_euler('xyz', degrees=True)
    euler_uzh = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])

    frames = list(range(1, len(acc_x_euroc)))
    plt.clf()
    #plt.plot(euler_euroc.x[frames], label='EuRoC euler X')
    #plt.plot(euler_euroc.y[frames], label='EuRoC euler Y')
    #plt.plot(euler_euroc.z[frames], label='EuRoC euler Z')
    plt.plot(raw_imu.timestamp.to_numpy(), euler_uzh.x.to_numpy(), label='UZH euler X')
    plt.plot(raw_imu.timestamp.to_numpy(), euler_uzh.y.to_numpy(), label='UZH euler Y')
    plt.plot(raw_imu.timestamp.to_numpy(), euler_uzh.z.to_numpy(), label='UZH euler Z')
    plt.legend()
    plt.title('Euler Angles Calculated from UZH dataset IMU Single Alone')
    plt.show()

