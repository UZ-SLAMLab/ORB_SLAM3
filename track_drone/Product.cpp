//
// Created by firas on 27.7.2020.
//

#include "Product.h"
#include <iostream>
#include <fstream>
#include "Utils.h"
#include <opencv2/core.hpp>
using namespace std;
Product::Product(const std::string& barcode,bool withcolor,const std::string& folder_path) {
    string mapERROR="wrong 3d map file\n";
    std::vector <cv::Point3f> tmp_3d_map;
    std::vector <cv::Scalar> tmp_colors;
    cv::Mat tmp_descriptors;
    std::string x,y,z,r,g,b;
    std::ifstream point3dfile,colorfile;
    cv::FileStorage descriptorsfile;
    std::cout << "Reading:\n";
//    descriptorsfile.open("../Barilla_very_new/descriptors_"+ barcode+".xml", cv::FileStorage::READ);
//    8000 100 0.6 8076809513388 /home/firas/CLionProjects/Daivitests/barilla_new/8076809529433/4/3/event_945_camera_3.mkv Test_results.csv /home/firas/CLionProjects/Daivitests/barilla_new
//    descriptorsfile.open("../point_clouds/cam_26020400_f_6/"+ barcode+"_descriptors"+".xml", cv::FileStorage::READ);
    descriptorsfile.open(folder_path+ barcode+"_descriptors"+".xml", cv::FileStorage::READ);
    try {
        if (!descriptorsfile.isOpened())
        {
            throw 15;
        }
        std::cout << "desc File is opened\n";
    }
    catch ( int err) {
        std::cout << "wrong descriptors File !!!! \n"<< err;

    }
    cv::Mat pose2;
//    point3dfile.open("../Barilla_very_new/point_"+ barcode+".csv");
    point3dfile.open(folder_path+ barcode+"_points"+".csv");
    if (withcolor)
        colorfile.open(folder_path+ barcode+"_colors"+".csv");
    else
        colorfile.open(folder_path+ barcode+"_points"+".csv");

    try {
        if (!point3dfile.is_open()) {
            throw mapERROR;
        }
        std::cout << " 3d-P File is opened\n";

    }
    catch (string mapERROR) {
        std::cout << mapERROR;
    }
    int i = 0;
    int counter = 0;
    while (point3dfile.good()) {
        switch (counter){
            case 0:
                getline(point3dfile, x, ',');
                getline(colorfile, r, ',');
                counter++;
                break;
            case 1:
                getline(point3dfile, y, ',');
                getline(colorfile, g, ',');
                counter++;
                break;
            case 2:
                std::string st = "desc" + std::to_string(i);
                descriptorsfile[st] >> pose2;
                getline(point3dfile, z, '\n');
                getline(colorfile, b, '\n');
                float xF = std::atof(x.c_str());
                float yF = std::atof(y.c_str());
                float zF = std::atof(z.c_str());
                float rF = std::atof(r.c_str());
                float gF = std::atof(g.c_str());
                float bF = std::atof(b.c_str());
                if(rF>245&&gF>245&&bF>245){continue;}
                tmp_descriptors.push_back({ pose2 });
                tmp_3d_map.emplace_back( xF, yF, zF );
                tmp_colors.emplace_back(cv::Scalar( rF, gF, bF) );
                counter = 0;
                i++;
                break;
        }
    }
//    vector <cv::DMatch> good_matches;
//    cv::Ptr <cv::FeatureDetector> detector, descriptor;
//    RobustMatcher rmatcher;
//    rmatcher.setFeatureDetector(detector);                                // set feature detector
//    rmatcher.setDescriptorExtractor(
//            descriptor);                          // set descriptor extractor
//    rmatcher.setDescriptorMatcher(createMatcher("ORB", true));     // set matcher
//    rmatcher.setRatio(0.98);
//    std::vector<std::vector<cv::DMatch> > matches;
//    std::vector<int> delete_indx;
//    std::vector<int> delete_indx2;
//    rmatcher.fastRobustMatch3(good_matches, tmp_descriptors, tmp_descriptors);
//    for (unsigned int match_index = 0; match_index < good_matches.size(); ++match_index) {
//        delete_indx.push_back(good_matches[match_index].trainIdx);
//    }
//
//    for (int j = 0; j <tmp_descriptors.rows ; j++) {
//        auto endofdata=find(delete_indx.begin(),delete_indx.end(), j);
//        if(endofdata!=delete_indx.end()) {
//            continue;
//        }
//        else {
//            products_descriptors_.push_back(tmp_descriptors.row(j));
//            products_3d_map_.emplace_back(tmp_3d_map[j]);
//            products_colors_.emplace_back(tmp_colors[j]);
//        }
//    }
    products_descriptors_=tmp_descriptors;
    products_3d_map_=tmp_3d_map;
    products_colors_=tmp_colors;
    barcode_=barcode;
    descriptorsfile.release();
    point3dfile.close();
    colorfile.close();
}
