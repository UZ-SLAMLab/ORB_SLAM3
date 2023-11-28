#!/bin/bash

# Check if both arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <DatasetPath> <Tracks>"
    echo "example: $0 $HOME/Datasets/SubPipe"
    exit 1
fi

pathDataset="$1"

echo "Dataset Path: $pathDataset"

./Monocular/mono_subpipe ../Vocabulary/ORBvoc.txt Monocular/SubPipe.yaml $pathDataset/DATA/ 
