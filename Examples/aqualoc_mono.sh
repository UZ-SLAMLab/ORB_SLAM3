#!/bin/bash
pathDatasetaqualoc='/home/olaya/Datasets/Aqualoc/Archaeological_site_sequences' #Example, it is necesary to change it by the dataset path

tracks='1 2 3 4 5 6'
for track in $tracks
do 
echo "Launching $track with Monocular sensor"
./Monocular/mono_aqualoc ../Vocabulary/ORBvoc.txt Monocular/Aqualoc.yaml "$pathDatasetaqualoc"/archaeo_sequence_$track\_raw_data $track
mv KeyFrameTrajectory.txt kf_archaeo_sequence_$track\_raw_data.txt
done

# ./Monocular/mono_aqualoc ../Vocabulary/ORBvoc.txt Monocular/Aqualoc.yaml /home/olaya/Datasets/Aqualoc/Archaeological_site_sequences/archaeo_sequence_1_raw_data 1