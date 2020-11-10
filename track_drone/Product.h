//
// Created by firas on 27.7.2020.
//
#include <opencv2/core.hpp>
#ifndef FORDR_DEV_PRODUCT_H
#define FORDR_DEV_PRODUCT_H

class Product {
public:
    Product(const std::string& barcode,bool withcolor,const std::string& folder_path);
    std::string barcode_;
    std::string name_; //redundant can be removed
    std::vector <cv::Point3f> products_3d_map_; // container for the model 3D coordinates found in the scene
    std::vector <cv::Scalar> products_colors_; // container for the model 3D coordinates found in the scene
    cv::Mat products_descriptors_;
    int number_of_frames_detected=0;
    float ratio_of_detected_frames=0;
    std::vector<cv::Point2f> tracked2dpoints[2];
    std::vector<cv::Point3f> tracked3dpoints[2];
    std::vector<cv::Point3f> trackedlines[2];
    std::vector<cv::Mat> trackedlines4coreset[2];
    double prevsumofcosts=std::numeric_limits<double>::infinity();;
    double sumofcosts=std::numeric_limits<double>::infinity();
    // ---------------------------------
};
#endif //FORDR_DEV_PRODUCT_H
