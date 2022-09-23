#!/bin/bash  
echo "Environment: $1";
echo "Track: $2";

./Examples/Stereo/stereo_mimir /home/olaya/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt Examples/Stereo/MIMIR.yaml $HOME/Datasets/MIMIR/$1/$2 $HOME/Datasets/MIMIR/$1/$2/auv0/ORB_timestamps.txt /home/olaya/Datasets/MIMIR/$1/$2 /home/olaya/Datasets/MIMIR/$1/$2/auv0/ORB_timestamps.txt $1_$2_cam2.txt