//
// Created by firas on 10/6/20.
//

#ifndef DAIVITESTS_PROJECTION_H
#define DAIVITESTS_PROJECTION_H

#include <opencv2/core.hpp>

class Projection {
public:
    Projection(const cv::Mat& R,const cv::Mat& t,const cv::Mat& K);
    void set_P_matirx(const cv::Mat R,const cv::Mat t);
    std::vector<cv::Point2f> verify_points(std::vector <cv::Point3f> map);
    cv::Point2f backproject3DPoint(const cv::Point3f &point3d);


private:
    /** The calibration matrix */
    cv::Mat K_matrix_;
    /** The computed rotation matrix */
    cv::Mat R_matrix_;
    /** The computed translation matrix */
    cv::Mat t_matrix_;
    /** The computed projection matrix */
    cv::Mat P_matrix_;
};



#endif //DAIVITESTS_PROJECTION_H
