#!/bin/bash

# Check if both arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <DatasetPath> <Tracks>"
    echo "example: $0 $HOME/Datasets/Aqualoc/Archaeological_site_sequences '1 2 3 4 5 6'"
    exit 1
fi

pathDatasetaqualoc="$1"
tracks="$2"

echo "Dataset Path: $pathDatasetaqualoc"
echo "Tracks: $tracks"

for track in $tracks
do
    echo "Launching $track with Monocular sensor"
    ./Monocular/mono_aqualoc ../Vocabulary/ORBvoc.txt Monocular/Aqualoc.yaml "$pathDatasetaqualoc/archaeo_sequence_$track\_raw_data" $track
    mv KeyFrameTrajectory.txt "kf_archaeo_sequence_$track\_raw_data.txt"
done