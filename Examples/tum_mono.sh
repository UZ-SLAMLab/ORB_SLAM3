#!/bin/bash

# Check if all three arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <datasetPath>"
    echo "example: $0 $HOME/Datasets/TUM"
    exit 1
fi

pathDatasetTUM="$1" 
echo "Dataset Path: $pathDatasetTUM"

#------------------------------------
# Monocular Examples
track="rgbd_dataset_freiburg1_360"
echo "Launching $track with Monocular sensor"
./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM1.yaml "$pathDatasetTUM"/Handheld/$track
mv KeyFrameTrajectory.txt kf_$track.txt

track="rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
echo "Launching $track with Monocular sensor"
./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM1.yaml "$pathDatasetTUM"/StructureTexture/$track
mv KeyFrameTrajectory.txt kf_$track.txt

track="rgbd_dataset_freiburg3_nostructure_notexture_far"
echo "Launching $track with Monocular sensor"./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM3.yaml "$pathDatasetTUM"/StructureTexture/$track
mv KeyFrameTrajectory.txt kf_$track.txt

track="rgbd_dataset_freiburg1_rpy"
echo "Launching $track with Monocular sensor"
./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM3.yaml "$pathDatasetTUM"/TestingDebugging/$track
mv KeyFrameTrajectory.txt kf_$track.txt

############################################################################
# echo "Launching Room 4 with Monocular sensor"
# ./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM1.yaml "$pathDatasetTUM"/dataset-room4_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_mono

# echo "Launching Room 5 with Monocular sensor"
# ./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM1.yaml "$pathDatasetTUM"/dataset-room5_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room5_512.txt dataset-room5_512_mono

# echo "Launching Room 6 with Monocular sensor"
# ./Monocular/mono_tum ../Vocabulary/ORBvoc.txt Monocular/TUM1.yaml "$pathDatasetTUM"/dataset-room6_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room6_512.txt dataset-room6_512_mono

