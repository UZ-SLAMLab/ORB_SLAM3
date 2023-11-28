#!/bin/bash  
echo "Environment: $1";
echo "Track: $2";
echo "camera: $3";

./Examples/Monocular/mono_mimir Vocabulary/ORBvoc.txt Examples/Monocular/MIMIR.yaml $HOME/Datasets/MIMIR/$1/$2/auv0/rgb/$3/data Examples/Monocular/MIMIR_TimeStamps/$1_$2.txt $1_$2_$3.txt


