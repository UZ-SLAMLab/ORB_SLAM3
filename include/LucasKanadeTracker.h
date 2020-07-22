//*****************************************************************
// File:    LucasKanadeTracker.h
// Author:  Juan José Gómez Rodríguez (jjgomez96@hotmail.com)
// Date:    01/05/2019
// Coms:    Header file of the Lucas-Kanade optical flow algorithm
//*****************************************************************

#ifndef KLT_H
#define KLT_H

#include<opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

class LucasKanadeTracker {
public:
    /*
     * Default constructor
     */
    LucasKanadeTracker();

    /*
     * Constructor with parameters
     */
    LucasKanadeTracker(const cv::Size _winSize, const int _maxLevel, const int _maxIters,
                       const float _epsilon, const float _minEigThreshold);

    /*
     *
     */
    void SetReferenceImage(cv::Mat &refIm, std::vector<cv::Point2f> &refPts);

    int PRE_Track(cv::Mat &newIm,
                   std::vector<cv::Point2f> &nextPts, std::vector<bool> &status, const bool bInitialFlow,
                   const bool bCheckLength);

    void DeleteRefPt(const int idx);
    cv::Point2f GetPrevPoint(const int idx);

private:
    //-------------------------------
    //        KLT parameters
    //-------------------------------
    cv::Size winSize;      //size of the integration window of the Lucas-Kanade algorithm
    int maxLevel;           //max level of the image pyramids
    int maxIters;           //max number of iterations of the optical flow algorithm
    float epsilon;          //minimum optical flow imposed displacement. If lower, we stop computing
    float minEigThreshold;  //min eigen threshold value for the Spatial Gradient matrix

    //-------------------------------
    //      Pre computed stuff
    //-------------------------------
    std::vector<cv::Point2f> prevPts;           //Original coordinates of the points to track
    std::vector<cv::Mat> refPyr;                //Reference pyramid
    std::vector<std::vector<float>> vMeanI;     //Reference window mean intensities
    std::vector<std::vector<float>> vMeanI2;    //Reference window mean sqared intensities
    std::vector<std::vector<cv::Mat>> Iref;     //Reference windows
    std::vector<std::vector<cv::Mat>> Idref;    //Reference derivative windows
};

#endif //KLT_H
