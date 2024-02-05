#!/bin/bash

# Check if all three arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <datasetPath>"
    echo "example: $0 $HOME/Datasets/KITTI"
    exit 1
fi

pathDataset="$1" 
pathDatasetKITTI=$pathDataset/data_odometry_gray/dataset/sequences #Example, it is necesary to change it by the dataset path
echo "Dataset Path: $pathDatasetKITTI"

#------------------------------------
# Monocular Examples
sequence="00"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI00-02.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="01"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI00-02.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="02"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI00-02.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="03"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI03.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="04"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="05"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="06"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="07"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="08"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="09"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt

sequence="10"
echo "Launching $sequence with Monocular sensor"
./Monocular/mono_kitti ../Vocabulary/ORBvoc.txt Monocular/KITTI04-12.yaml $pathDatasetKITTI/$sequence
mv KeyFrameTrajectory.txt kf_KITTI_$sequence.txt
