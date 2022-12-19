echo "Building ROS nodes"

cd /dpds/ORB_SLAM3/Examples/ROS/ORB_SLAM3
mkdir build
cd build

cmake .. -DROS_BUILD_TYPE=Release
make 
