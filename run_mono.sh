#!/bin/bash  
echo "Environment: $1";
echo "Track: $2";
echo "camera: $3";

./Examples/Monocular/mono_mimir Vocabulary/ORBvoc.txt Examples/Monocular/MIMIR.yaml $HOME/Datasets/MIMIR/$1/$2/auv0/rgb/$3/data $HOME/Datasets/MIMIR/$1/$2/auv0/ORB_timestamps.txt $1_$2_$3.txt


