# Some bugs in ORB-SLAM3
## Changed Codes
#### SearchForTriangulation 
https://github.com/fishmarch/ORB_SLAM3/blob/b66d0b7eedfb363e5bad0cee8ea12065f2d66ae8/src/ORBmatcher.cc#L1085

Forgot to change flags to label matched features.

This would cause multiple keypoints in KF1 are matched with same keypoint in KF2; and then all these new map points contain obervations KF1, but KF1 only records one of them.

#### UpdateConnections
https://github.com/fishmarch/ORB_SLAM3/blob/b66d0b7eedfb363e5bad0cee8ea12065f2d66ae8/src/KeyFrame.cc#L427-L443

Here caused unidirectional connection. 

When updating connection of KF, all other keyframes sharing co-observed mappoints are collected. Only when the weights (number of co-obervations) are larger than a threshold (15), the connections are recorded bidirectionally; otherwise, the records should be erased.

#### Sim3Solver
https://github.com/fishmarch/ORB_SLAM3/blob/b66d0b7eedfb363e5bad0cee8ea12065f2d66ae8/src/Sim3Solver.cc#L40-L43

The flag was set wrong here.

This would cause the index cannot be found properly later, and then less matched map points are collected.

#### DetectNBestCandidates
https://github.com/fishmarch/ORB_SLAM3/blob/b66d0b7eedfb363e5bad0cee8ea12065f2d66ae8/src/KeyFrameDatabase.cc#L658-L664

Only computed scores for keyframes in which the number of common words is larger than a threshold, but the scores of other keyframes may also be used later. 

https://github.com/fishmarch/ORB_SLAM3/blob/b66d0b7eedfb363e5bad0cee8ea12065f2d66ae8/src/KeyFrameDatabase.cc#L710-L726

The iterator was not updated when the keyframe is bad.









