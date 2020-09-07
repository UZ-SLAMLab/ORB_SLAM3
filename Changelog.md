# ORB-SLAM3
Details of changes between the different versions.

### V0.3: Beta version, 4 Sep 2020

- RGB-D compatibility, the RGB-D examples had been adapted to the new version.

- Kitti and TUM dataset compatibility, these examples had been adapted to the new version.

- ROS compatibility, It had been updated the old references in the code to work with this version.

- Config file parser, the YAML file contains the session configuration, a wrong parametrization may break the execution without any information to solve it. This version parses the file to read all the fields and give a proper answer if one of the fields have been wrong deffined or don't exist.

- Fixed minor bugs.


### V0.2: Beta version, 7 Aug 2020
Initial release. It has these capabilities:

- Multiple-Maps capabilities, it is able to handle multiple maps in the same session and merge them when a common area is detected with a seamless fussion.

- Inertial sensor, the IMU initialization takes a 2 seconds to achieve a scale error less than 5\% and it is reffined in the next 10 seconds until is around 1\%. Inertial measures are integrated at frame rate to estimate the scale, gravity and velocity in order to improve the visual features detection and make the system robust to temporal occlusions.

- Fisheye sensor, the fisheye sensors are now fully supported in monocular and stereo. 


