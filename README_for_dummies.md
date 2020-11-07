# Install
- installing opencv from debian packages is good enough, try using:
```
apt-get -y install libopencv-dev python3-opencv
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
       
## for raspi_cam
install raspi_cam camera node