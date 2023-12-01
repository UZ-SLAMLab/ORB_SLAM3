import cv2
import pandas as pd

if __name__ == "__main__":

    video_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/1701402143748.mp4'
    images_save_directory = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0/'
    raw_acceleration_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/AccelerometerUncalibrated.csv'
    raw_gyro_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/GyroscopeUncalibrated.csv'
    images_timestamp_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0.txt'
    images_timestamp_names_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/Camera/Camera0.csv'
    joined_imu_path = '/home/justmohsen/Documents/SLAM/Datasets/Pixel6/HomeFlying/Concorde_Place-2023-11-30_03-42-23/joinedRawIMU.csv'
    video_clip = cv2.VideoCapture(video_path)
    fps = video_clip.get(cv2.CAP_PROP_FPS)
    success, image = video_clip.read()
    count = 0
    initial_timestamp = 1701402143748000000
    time_elapsed = 0
    image_timestamp = []
    image_names = []

    while success:
        timestamp_str = str(initial_timestamp + round(time_elapsed))
        name = timestamp_str + ".jpg"
        cv2.imwrite(images_save_directory + name, image)
        image_timestamp.append(timestamp_str)
        image_names.append(name)
        success, image = video_clip.read()
        count += 1
        time_elapsed += 1e9 / fps

    images_names_timestamp = pd.DataFrame(columns=['image_timestamp', 'image_names'])
    images_names_timestamp.image_timestamp = image_timestamp
    images_names_timestamp.image_names = image_names
    images_names_timestamp.to_csv(images_timestamp_names_path, header=False, index=False, float_format='%d')
    images_timestamp = pd.DataFrame(columns=['image_timestamp'])
    images_timestamp.image_timestamp = image_timestamp
    images_timestamp.to_csv(images_timestamp_path, header=False, index=False)
    # now combine imu
    raw_gyro = pd.read_csv(raw_gyro_path)
    raw_gyro = raw_gyro.rename(columns={"time": "#timestamp [ns]", "x": "w_RS_S_x [rad s^-1]", "y": "w_RS_S_y [rad s^-1]", "z": "w_RS_S_z [rad s^-1]"})
    raw_acceleration = pd.read_csv(raw_acceleration_path)
    raw_acceleration = raw_acceleration.rename(columns={"time": "#timestamp [ns]", "x": "a_RS_S_x [m s^-2]", "y": "a_RS_S_y [m s^-2]", "z": "a_RS_S_z [m s^-2]"})
    raw_imu = raw_gyro.merge(raw_acceleration, left_on='#timestamp [ns]', right_on='#timestamp [ns]', how='inner')
    raw_imu = raw_imu.loc[:,
              ['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]']]
    raw_imu.to_csv(joined_imu_path, header=True, index=False)
    print(f'Done reading and writing {count} images')
