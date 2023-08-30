#! /bin/bash

## run ORBSLAM3 mono on euroc dataset

path2ORBSLAM3=/home/zmxj/code/ORB_SLAM3
path2eurocdataset=/home/zmxj/code/Datasets/Euroc/MH01
outputfile=dataset-MH01_mono

cd $path2ORBSLAM3
./Examples/Monocular/mono_euroc $path2ORBSLAM3/Vocabulary/ORBvoc.txt $path2ORBSLAM3/Examples/Monocular/EuRoC.yaml $path2eurocdataset $path2ORBSLAM3/Examples/Monocular/EuRoC_TimeStamps/MH01.txt $outputfile


