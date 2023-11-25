echo "Building ROS2 nodes"

cd Examples/ROS2/ORB_SLAM3
rm -rf build
rm -rf install
rm -rf log
source /opt/ros/foxy/setup.bash
colcon build --packages-select orb_slam3_ros2
. install/setup.bash
