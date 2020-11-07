# Install
- installing opencv from debian packages is good enough, try using:
```
apt-get -y install libopencv-dev python3-opencv
```
- eigen
```
sudo apt-get -y install libeigen3-dev libblas-dev liblapack-dev libglew-dev
```

...


# ros install
- follow ros official instructions
according to origina orb-slam3 repo they were running on ros melodic (ubuntu18)

# setup ros project/working directory
```shell script
mkdir -p catkin_ws/src 
cd catkin_ws
catkin_make
```
all of the nodes code should be stored under `catkin_ws/src` folder
after every code update there is a need to recompile the node packages
to do so run `catkin_make`

# test your camera setup
before starting to work on any project first load ros project parameters in relevant terminal window
`cd catkin_ws`
and then run
```
source devel/setup.bash
```
for zsh
```
source devel/setup.zsh
```

Now you can start using ROS

for example, lets test our camera setup:
## for usb web-cam
- install camera node:
```
sudo apt-get install ros-melodic-cv-camera
```
- to test web cam run following commands:
    1. to start ros server:
    ```roscore```
    2. to start camera publisher node: 
        ```
        rosparam set cv_camera/device_id 0
        rosrun cv_camera cv_camera_node  
        ```
        change device_id to you camera device id, usually on linux machines the numbering starts from 0
        where 0 is built in camera
    3. to start camera viewer
        ```
        rosrun image_view image_view image:=/cv_camera/image_raw
        ```
        image_view can show multiple cameras, to change camera switch topic
        to check which topic run
        ```
        rostopic list -v
        ```
- for more information on camera_view node http://wiki.ros.org/image_view
   
   this node can also record video feed, view compressed images (better for 
   over wifi transmission) and some more.
   
   camera recording has also a lot of settings, for full list
   visit http://wiki.ros.org/image_view
   
   
for better results run camera calibration with chessboard
http://wiki.ros.org/camera_calibration

       
## for raspi_cam
install raspi_cam camera node

```shell script

cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
cd src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/ros-perception/image_common.git

# to see and test camera
roslaunch raspicam_node camerav2_1280x960.launch &
rosrun raspicam_node imv_view.py
# additional info at git repo raspicam_node
```
# run ORB_SLAM3 with ROS and USB Web Cam on a Laptop
 - build the whole project from main cmake file. in clion just load cmake and press build
this will create libraries for thirdparty dependancies and orb_slam3
- unpack vocabulary file

add example path to ros package path
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/george/Documents/ORB_SLAM3_custom/Examples/ROS
```

it seems worked with this path:
```
export ROS_PACKAGE_PATH=/home/george/Documents/catkin_ws/src:/opt/ros/melodic/share:/home/george/Documents/ORB_SLAM3_custom:/home/george/Documents/ORB_SLAM3_custom/Examples/ROS
```

start ros server:
```roscore```
start camera:
```
rosrun usb_cam usb_cam_node
```
or 
```
osparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node
```
update mono.cc code accordingly
run example:
```
rosrun ORB_SLAM3 Mono /home/george/Documents/ORB_SLAM3_custom/Vocabulary/ORBvoc.txt /home/george/Documents/ORB_SLAM3_custom/Examples/ROS/ORB_SLAM3/Asus.yaml
```
