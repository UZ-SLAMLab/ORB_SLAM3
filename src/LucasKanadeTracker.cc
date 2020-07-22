//*****************************************************************
// File:    LucasKanadeTracker.cpp
// Author:  Juan José Gómez Rodríguez (jjgomez96@hotmail.com)
// Date:    01/05/2019
// Coms:    Implementation file of the Lucas-Kanade optical flow algorithm
//*****************************************************************

#include "LucasKanadeTracker.h"

#include <float.h>

using namespace std;
using namespace cv;

#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_DESCALE(x,n)     (((x) + (1 << ((n)-1))) >> (n))


LucasKanadeTracker::LucasKanadeTracker() : winSize(Size(21, 21)), maxLevel(3), maxIters(30), epsilon(0.01),
                                           minEigThreshold(1e-4) {}

LucasKanadeTracker::LucasKanadeTracker(const cv::Size _winSize, const int _maxLevel, const int _maxIters,
                                       const float _epsilon, const float _minEigThreshold) :
        winSize(_winSize), maxLevel(_maxLevel), maxIters(_maxIters), epsilon(_epsilon),
        minEigThreshold(_minEigThreshold) {}

void LucasKanadeTracker::SetReferenceImage(Mat &refIm, vector<Point2f> &refPts) {
    //Compute reference pyramid
    cv::buildOpticalFlowPyramid(refIm, refPyr, winSize, maxLevel);

    //Store points
    prevPts = refPts;

    vMeanI = vector<vector<float>>(maxLevel + 1, vector<float>(refPts.size()));
    vMeanI2 = vector<vector<float>>(maxLevel + 1, vector<float>(refPts.size()));
    Iref = vector<vector<Mat>>(maxLevel + 1, vector<Mat>(refPts.size()));
    Idref = vector<vector<Mat>>(maxLevel + 1, vector<Mat>(refPts.size()));

    //Compute reference windows (intensity and derivtives) and means of the windows
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    for (int level = maxLevel; level >= 0; level--) {
        //Get images form the pyramid
        const Mat I = refPyr[level * 2];
        const Mat derivI = refPyr[level * 2 + 1];

        //Steps for matrix indexing
        int dstep = (int) (derivI.step / derivI.elemSize1());
        int stepI = (int) (I.step / I.elemSize1());

        //Buffer for fast memory access
        int cn = I.channels(), cn2 = cn * 2;   //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2));
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + winSize.area() * cn);

        for (int i = 0; i < prevPts.size(); i++) {
            //Compute image coordinates in the reference image at the current level
            Point2f point = prevPts[i] / (1 << level);

            Point2i ipoint;
            point -= halfWin;
            ipoint.x = cvFloor(point.x);
            ipoint.y = cvFloor(point.y);

            if (ipoint.x < -winSize.width || ipoint.x >= derivI.cols ||
                    ipoint.y < -winSize.height || ipoint.y >= derivI.rows) {
                continue;
            }

            //Compute weighs for sub pixel computation
            float a = point.x - ipoint.x;
            float b = point.y - ipoint.y;
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);
            int iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
            int iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
            int iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
            int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI = 0.f, meanI2 = 0.f;

            int x, y;
            for (y = 0; y < winSize.height; y++) {
                //Get pointers to the images
                const uchar *src = I.ptr() + (y + ipoint.y) * stepI + ipoint.x;
                const short *dsrc = derivI.ptr<short>() + (y + ipoint.y) * dstep + ipoint.x * 2;

                //Get pointers to the window buffers
                short *Iptr = IWinBuf.ptr<short>(y);
                short *dIptr = derivIWinBuf.ptr<short>(y);

                x = 0;
                for (; x < winSize.width * cn; x++, dsrc += 2, dIptr += 2) {
                    //Get sub pixel values from images
                    int ival = CV_DESCALE(src[x] * iw00 + src[x + cn] * iw01 +
                                          src[x + stepI] * iw10 + src[x + stepI + cn] * iw11, W_BITS1 - 5);
                    int ixval = CV_DESCALE(dsrc[0] * iw00 + dsrc[cn2] * iw01 +
                                           dsrc[dstep] * iw10 + dsrc[dstep + cn2] * iw11, W_BITS1);
                    int iyval = CV_DESCALE(dsrc[1] * iw00 + dsrc[cn2 + 1] * iw01 + dsrc[dstep + 1] * iw10 +
                                           dsrc[dstep + cn2 + 1] * iw11, W_BITS1);

                    //Store values to the window buffers
                    Iptr[x] = (short) ival;
                    dIptr[0] = (short) ixval;
                    dIptr[1] = (short) iyval;

                    //Compute accum values for later gain and bias computation
                    meanI += (float) ival;
                    meanI2 += (float) (ival * ival);
                }
            }
            //Compute means for later gain and bias computation
            vMeanI[level][i] = (meanI * FLT_SCALE) / winSize.area();
            vMeanI2[level][i] = (meanI2 * FLT_SCALE) / winSize.area();

            Iref[level][i] = IWinBuf.clone();
            Idref[level][i] = derivIWinBuf.clone();
        }
    }

}

int LucasKanadeTracker::PRE_Track(Mat &newIm,
                                   std::vector<Point2f> &nextPts, vector<bool> &status, const bool bInitialFlow,
                                   const bool bCheckLength) {
    //Set status of all the points to true
    status = vector<bool>(prevPts.size(), true);

    if (!bInitialFlow)
        nextPts = vector<Point2f>(prevPts.size());

    //Dimensions of half of the window
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    //Compute pyramid images
    vector<Mat> newPyr;
    cv::buildOpticalFlowPyramid(newIm, newPyr, winSize, maxLevel);

    //Start Lucas-Kanade optical flow algorithm
    //First iterate over pyramid levels
    for (int level = maxLevel; level >= 0; level--) {
        //Get images and gradients
        const Mat I = refPyr[level * 2];
        const Mat J = newPyr[level * 2];
        const Mat derivI = refPyr[level * 2 + 1];
        const Mat derivJ = newPyr[level * 2 + 1];

        //Buffer for fast memory access
        int j, cn = I.channels(), cn2 = cn * 2;   //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2) * 2);
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat JWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf) + winSize.area());
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 2 * winSize.area());
        Mat derivJWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 4 * winSize.area());

        //Steps for matrix indexing
        int dstep = (int) (derivI.step / derivI.elemSize1());
        int stepJ = (int) (J.step / J.elemSize1());

        //Track each point at the current pyramid level
        for (int i = 0; i < prevPts.size(); i++) {
            //Compute image coordinates in the reference image at the current level
            Point2f prevPt = prevPts[i] * (float) (1. / (1 << level));
            //Compute image coordinates in the current frame at the current level
            Point2f nextPt;
            if (level == maxLevel) {
                if (bInitialFlow) {
                    nextPt = nextPts[i] * (float) (1. / (1 << level));
                } else {
                    nextPt = prevPt;
                }
            } else {
                nextPt = nextPts[i] * 2.f;
            }

            nextPts[i] = nextPt;

            //Check that previous point and next point is inside of the
            //image boundaries
            Point2i iprevPt, inextPt;
            prevPt -= halfWin;
            iprevPt.x = cvFloor(prevPt.x);
            iprevPt.y = cvFloor(prevPt.y);

            if (iprevPt.x < -winSize.width || iprevPt.x >= derivI.cols ||
                iprevPt.y < -winSize.height || iprevPt.y >= derivI.rows) {
                if (level == 0)
                    status[i] = false;

                continue;
            }

            //Compute weighs for sub pixel computation
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI = 0.f, meanI2 = 0.f;

            int x, y;
            //Compute means for later gain and bias computation
            meanI = vMeanI[level][i];
            meanI2 = vMeanI2[level][i];

            IWinBuf = Iref[level][i].clone();
            derivIWinBuf = Idref[level][i].clone();

            //Optical flow loop
            Point2f prevDelta;
            nextPt -= halfWin;
            for (j = 0; j < maxIters; j++) {
                //Compute weighs for sub pixel computation
                inextPt.x = cvFloor(nextPt.x);
                inextPt.y = cvFloor(nextPt.y);

                //Check that the point is inside the image
                if (inextPt.x < -winSize.width || inextPt.x >= J.cols ||
                    inextPt.y < -winSize.height || inextPt.y >= J.rows) {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                float aJ = nextPt.x - inextPt.x;
                float bJ = nextPt.y - inextPt.y;
                int jw00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
                int jw01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
                int jw10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
                int jw11 = (1 << W_BITS) - jw00 - jw01 - jw10;

                //Compute alpha and beta for gain and bias
                //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
                float meanJ = 0.f, meanJ2 = 0.f;

                for (y = 0; y < winSize.height; y++) {
                    //Get pointers to the images
                    const uchar *src = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;
                    const short *dsrc = derivJ.ptr<short>() + (y + inextPt.y) * dstep + inextPt.x * cn2;

                    //Get pointers to the window buffers
                    short *Jptr = JWinBuf.ptr<short>(y);
                    short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;

                    for (; x < winSize.width * cn; x++, dsrc += 2, dJptr += 2) {
                        //Get sub pixel values from images
                        int jval = CV_DESCALE(src[x] * jw00 + src[x + cn] * jw01 +
                                              src[x + stepJ] * jw10 + src[x + stepJ + cn] * jw11, W_BITS1 - 5);
                        int jxval = CV_DESCALE(dsrc[0] * jw00 + dsrc[cn2] * jw01 +
                                               dsrc[dstep] * jw10 + dsrc[dstep + cn2] * jw11, W_BITS1);
                        int jyval = CV_DESCALE(dsrc[1] * jw00 + dsrc[cn2 + 1] * jw01 + dsrc[dstep + 1] * jw10 +
                                               dsrc[dstep + cn2 + 1] * jw11, W_BITS1);

                        //Store values to the window buffers
                        Jptr[x] = (short) jval;
                        dJptr[0] = (short) jxval;
                        dJptr[1] = (short) jyval;

                        //Compute accum values for later gain and bias computation
                        meanJ += (float) jval;
                        meanJ2 += (float) (jval * jval);
                    }
                }

                //Compute means for later gain and bias computation
                meanJ = (meanJ * FLT_SCALE) / winSize.area();
                meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

                //Compute alpha and beta
                float alpha = sqrt(meanI2 / meanJ2);
                float beta = meanI - alpha * meanJ;

                //Compute image gradient insensitive to ilumination changes
                float ib1 = 0, ib2 = 0;
                float b1, b2;
                float iA11 = 0, iA12 = 0, iA22 = 0;
                float A11, A12, A22;

                for (y = 0; y < winSize.height; y++) {
                    //Get pointers to the buffers
                    const short *Iptr = IWinBuf.ptr<short>(y);
                    const short *Jptr = JWinBuf.ptr<short>(y);
                    const short *dIptr = derivIWinBuf.ptr<short>(y);
                    const short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;
                    for (; x < winSize.width * cn; x++, dIptr += 2, dJptr += 2) {
                        int diff = Jptr[x] * alpha - Iptr[x] - beta;
                        float dx = (float) (dIptr[0] + dJptr[0] * alpha);
                        float dy = (float) (dIptr[1] + dJptr[1] * alpha);

                        ib1 += (float) (diff * dx);
                        ib2 += (float) (diff * dy);

                        iA11 += (float) (dx * dx);
                        iA22 += (float) (dy * dy);
                        iA12 += (float) (dx * dy);
                    }
                }
                b1 = ib1 * FLT_SCALE;
                b2 = ib2 * FLT_SCALE;

                //Compute spatial gradient matrix
                A11 = iA11 * FLT_SCALE;
                A12 = iA12 * FLT_SCALE;
                A22 = iA22 * FLT_SCALE;

                float D = A11 * A22 - A12 * A12;
                float minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) +
                                                      4.f * A12 * A12)) / (2 * winSize.width * winSize.height);

                if (minEig < minEigThreshold || D < FLT_EPSILON) {
                    if (level == 0)
                        status[i] = false;
                    continue;
                }

                D = 1.f / D;

                //Compute optical flow
                Point2f delta((float) ((A12 * b2 - A22 * b1) * D),
                              (float) ((A12 * b1 - A11 * b2) * D));

                nextPt += delta;
                nextPts[i] = nextPt + halfWin;

                if (delta.ddot(delta) <= epsilon)
                    break;

                if (j > 0 && std::abs(delta.x + prevDelta.x) < 0.01 &&
                    std::abs(delta.y + prevDelta.y) < 0.01) {
                    nextPts[i] -= delta * 0.5f;
                    break;
                }
                prevDelta = delta;
            }
        }
    }

    //Track lenght check
    if (bCheckLength) {
        //Compute total displacement of aeach good tracked point
        vector<float> deltas, deltasShorted;
        for (int i = 0; i < prevPts.size(); i++) {
            if (!status[i]) {
                deltas.push_back(0.f);
                continue;
            }
            Point2f pDelta = nextPts[i] - prevPts[i];
            deltas.push_back(sqrt(pDelta.x * pDelta.x + pDelta.y * pDelta.y));
            deltasShorted.push_back(sqrt(pDelta.x * pDelta.x + pDelta.y * pDelta.y));
        }
        sort(deltasShorted.begin(), deltasShorted.end());
        float median = deltasShorted[deltasShorted.size() / 2];

        if(median < 20.f){
            return deltasShorted.size();
        }

        for (int i = deltas.size() - 1; i >= 0; i--) {
            if (deltas[i] > median * 2)
                status[i] = false;
        }
        return deltasShorted.size();
    }
}

void LucasKanadeTracker::DeleteRefPt(const int idx)
{
    prevPts.erase(prevPts.begin()+idx);
    for(int level = maxLevel; level >= 0; level--)
    {
        vMeanI[level].erase(vMeanI[level].begin()+idx);
        vMeanI2[level].erase(vMeanI2[level].begin()+idx);
        Iref[level].erase(Iref[level].begin()+idx);
        Idref[level].erase(Idref[level].begin()+idx);

    }
}

cv::Point2f LucasKanadeTracker::GetPrevPoint(const int idx)
{
    return prevPts[idx];
}
