/*
 * Utils.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include <iostream>
#include <fstream>

#include "PnPProblem.h"
#include "Utils.h"

#include <opencv2/opencv_modules.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/flann.hpp>
#if defined (HAVE_OPENCV_XFEATURES2D)
#include <opencv2/xfeatures2d.hpp>
#endif

// For text
const int fontFace = cv::FONT_ITALIC;
const double fontScale = 0.75;
const int thickness_font = 2;

// For circles
const int lineType = 8;
const int radius = 4;

// Draw a text with the question point

using namespace cv;
using namespace std;

void optimalpnp(std::vector<Point3f> list_points3d_inliers,std::vector<Point3f> list_lines_inliers,int modelindex,float *Rvect,float *tvect,std::string datapath)
{
    int numofinliers=list_points3d_inliers.size();

    string matstring=datapath+"Mattest"+IntToString(modelindex)+".txt";
    ofstream matfile;
    matfile.open (matstring);
    matfile << IntToString(numofinliers)<<" 9 30\n";
    for (int i = 0; i < numofinliers; i++)
    {   
        matfile << format("%f %f %f ", list_lines_inliers[i].x, list_lines_inliers[i].y,list_lines_inliers[i].z)<<" "<<"0 0 0 "<<
                format("%f %f %f ", list_points3d_inliers[i].x, list_points3d_inliers[i].y,list_points3d_inliers[i].z)<<"\n";
    }
    matfile << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 \n";
    for (int i = 0; i < numofinliers; i++)
    {matfile << "1 ";}
    matfile.close();
    cout<<modelindex<<endl;

    int ret=system(("/home/firas/opencv/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/Data/./pnp_solver_sdpa "+ matstring+" OPnP_R_"+IntToString(modelindex)+".txt "+ "OPnP_t_"+IntToString(modelindex)+".txt >/dev/null").c_str());

    cout<<ret<<endl;
    string line;
    const char delim = ',';
    int row=0;
    ifstream rotfile ("/home/firas/opencv/build/bin/OPnP_R_"+IntToString(modelindex+1)+".txt");
    ifstream transfile ("/home/firas/opencv/build/bin/OPnP_t_"+IntToString(modelindex+1)+".txt");

    int couni=0;
    if (rotfile.is_open()){
        while ( getline (rotfile,line) )
        {
            std::vector<std::string> out;
            split(line, delim, out);
            int col =0 ;
            for (auto &s: out) {
                double temp = (double)atof(s.c_str());
                Rvect[couni]=temp;
                couni+=1;
                col+=1;    
                }
            row+=1;

        }
        
        rotfile.close();
    }

    row=0;
    if (transfile.is_open()){
        while ( getline (transfile,line) )
        {
            std::vector<std::string> out;
            split(line, delim, out);
            for (auto &s: out) {
                double temp = (double)atof(s.c_str());
                tvect[row]=temp;
                row+=1;
                }
        }
        transfile.close();

    }
}


void detect_object_per_frame( cv::Mat& frame,std::vector<cv::DMatch> *good_matches,std::vector<cv::KeyPoint> *keypoints_scene,
                        int modelindex,RobustMatcher *rmatcher, cv::Mat *frame_matching,vector<Point3f> *list_points3d_model_match,
                        vector<Point2f> *list_points2d_scene_match, Point3f *point3d_model,Point2f *point2d_scene, cv::Mat *descriptors_model
                        , vector<Point3f> *list_points3d_model, cv::Mat *inliers_idx,std::vector<Point2f> *list_points2d_inliers,std::vector<Point3f> *list_points3d_inliers,std::vector<Point3f> *list_lines_inliers, int *inliers_int,
                        int *outliers_int,string *inliers_str, string *outliers_str,PnPProblem pnp_detection,int pnpMethod,int iterationsCount,
                        float reprojectionError,double confidence,float *sumofcosts, string datapath 
                       )
    {

       
                                
        float values[9]={ 1.74569971e+03, 0 ,6.24717240e+02,0, 1.77817395e+03, 7.45964999e+02, 0, 0, 1 };
        // float values[9]={ 1.13564893e+03, 0, 8.17670593e+02, 0, 8.36369383e+02, 7.00403948e+02, 0, 0, 1 };
        cv::Mat K(3,3,CV_32FC1,values);
        
        // Some basic colors
        Scalar red(0, 0, 255);
        Scalar green(0,255,0);
        Scalar blue(255,0,0);
        Scalar yellow(0,255,255);


        cv::Mat frame_vis = frame.clone();    // refresh visualisation frame

        rmatcher[modelindex].fastRobustMatch(frame, good_matches[modelindex], keypoints_scene[modelindex], descriptors_model[modelindex]);
        frame_matching[modelindex] = rmatcher[modelindex].getImageMatching();
        
        for(unsigned int match_index = 0; match_index < good_matches[modelindex].size(); ++match_index)
        {
            point3d_model[modelindex] = list_points3d_model[modelindex][ good_matches[modelindex][match_index].trainIdx ];  // 3D point from model
            point2d_scene[modelindex] = keypoints_scene[modelindex][ good_matches[modelindex][match_index].queryIdx ].pt; // 2D point from the scene
            list_points3d_model_match[modelindex].push_back(point3d_model[modelindex]);         // add 3D point
            list_points2d_scene_match[modelindex].push_back(point2d_scene[modelindex]);         // add 2D point
        }
     

        if(good_matches[modelindex].size() >= 4){
        pnp_detection.estimatePoseRANSAC( list_points3d_model_match[modelindex], list_points2d_scene_match[modelindex],
                                          pnpMethod, inliers_idx[modelindex],
                                          iterationsCount, reprojectionError, confidence );
        }
        // cout<<"point2d"<<endl;
        // -- Step 4: Catch the inliers keypoints to draw
        for(int inliers_index = 0; inliers_index < inliers_idx[modelindex].rows; ++inliers_index)
        {
            int n = inliers_idx[modelindex].at<int>(inliers_index);         // i-inlier
            Point2f point2d = list_points2d_scene_match[modelindex][n];     // i-inlier point 2D
            
            Point3f point3d = list_points3d_model_match[modelindex][n];     // i-inlier point 3D
           
            list_points2d_inliers[modelindex].push_back(point2d);           // add i-inlier to list
            list_points3d_inliers[modelindex].push_back(point3d);          // add i-inlier to list
            cv::Mat ptMat = (cv::Mat_<float>(3,1)<<point2d.x, point2d.y, 1);
            Mat congo=K.inv();
            cv::Mat matline= congo*ptMat;
            matline=matline/norm(matline);
            Point3f v(matline) ;
            list_lines_inliers[modelindex].push_back(v);

        }
        float sumcost=0;

        Mat I= Mat::eye(3, 3, CV_32FC1);
        vector<cv::Mat> alllinesprod;
        bool isinliers=false;
        int numinliers=list_points3d_inliers[modelindex].size();
        // cout<<"assssssssssssdasdffffffffffff"<<endl;
        // cout<<list_points3d_inliers[modelindex].size()<<endl;
        if (numinliers>4)
        {
            
            float rv[9];
            float tv[3];
            optimalpnp(list_points3d_inliers[modelindex],list_lines_inliers[modelindex],modelindex,rv,tv,datapath );
            rv[1]=-rv[1];
            cv::Mat rot(3,3,CV_32F,rv);
            cv::Mat trans(3,1,CV_32F,tv);
            for (int indx = 0; indx < numinliers; indx++){
                Mat vtv = Mat(list_lines_inliers[modelindex][indx])*Mat(list_lines_inliers[modelindex][indx]).t();
                Mat Rpminust=((rot*Mat(list_points3d_inliers[modelindex][indx]))-trans).t()*(I-vtv);
                float cost=norm(Rpminust)*norm(Rpminust);
                isinliers=true;
                sumcost+=(cost/numinliers);
            }
            if(isinliers==true){
                if(isnan(sumcost) ){sumofcosts[modelindex]=0;}
                else{sumofcosts[modelindex]=sumcost;}
                
            }
            else{sumofcosts[modelindex]=std::numeric_limits<double>::infinity(); }
        }
        else{sumofcosts[modelindex]=std::numeric_limits<double>::infinity(); }
        inliers_int[modelindex] = inliers_idx[modelindex].rows;
        outliers_int[modelindex] = (int)good_matches[modelindex].size() - inliers_int[modelindex];
        inliers_str[modelindex] = IntToString(inliers_int[modelindex]);
        outliers_str[modelindex] = IntToString(outliers_int[modelindex]);

    }




void split(std::string const &str, const char delim,
            std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

void drawQuestion(cv::Mat image, cv::Point3f point, cv::Scalar color)
{
    std::string x = IntToString((int)point.x);
    std::string y = IntToString((int)point.y);
    std::string z = IntToString((int)point.z);

    std::string text = " Where is point (" + x + ","  + y + "," + z + ") ?";
    cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,850), fontFace, 1.3, color, 3, 8);
}

// Draw a text with the number of entered points
void drawText2(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,105), fontFace, fontScale, color, thickness_font, 8);
}

void drawText3(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,35), fontFace, 1., color, 3, 8);
}

void drawText4(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,125), fontFace, fontScale, color, thickness_font, 8);
}

void drawText5(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,700), fontFace, 1.0, color, 3, 8);
}
void drawText7(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,800), fontFace, 1.2, color, 3, 8);
}
void drawText6(cv::Mat image, std::string text, cv::Scalar color)
{
    cv::putText(image, text, cv::Point(25,650), fontFace, 1.3, color, 3, 8);
}


// Draw a text with the frame ratio
void drawFPS(cv::Mat image, double fps, cv::Scalar color)
{
    std::string fps_str = cv::format("%.2f FPS", fps);
    cv::putText(image, fps_str, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the frame ratio
void drawConfidence1(cv::Mat image, double confidence, cv::Scalar color)
{
    std::string conf_str = IntToString((int)confidence);
    std::string text = conf_str + " %";
    cv::putText(image, text, cv::Point(500,900), fontFace, 1.3, color, 3, 8);
}

void drawConfidence2(cv::Mat image, double confidence, cv::Scalar color)
{
    std::string conf_str = IntToString((int)confidence);
    std::string text = conf_str + " %";
    cv::putText(image, text, cv::Point(500,1000), fontFace, 1.3, color, 3, 8);
}

// Draw a text with the number of entered points
void drawCounter(cv::Mat image, int n, int n_max, cv::Scalar color)
{
    std::string n_str = IntToString(n);
    std::string n_max_str = IntToString(n_max);
    std::string text = n_str + " of " + n_max_str + " points";
    cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw the points and the coordinates
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color)
{
    for (unsigned int i = 0; i < list_points_2d.size(); ++i)
    {
        cv::Point2f point_2d = list_points_2d[i];
        cv::Point3f point_3d = list_points_3d[i];

        // Draw Selected points
        cv::circle(image, point_2d, radius, color, -1, lineType );

        std::string idx = IntToString(i+1);
        std::string x = IntToString((int)point_3d.x);
        std::string y = IntToString((int)point_3d.y);
        std::string z = IntToString((int)point_3d.z);
        std::string text = "P" + idx + " (" + x + "," + y + "," + z +")";

        point_2d.x = point_2d.x + 10;
        point_2d.y = point_2d.y - 10;
        cv::putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
    }
}

// Draw only the 2D points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color)
{
    for( size_t i = 0; i < list_points.size(); i++)
    {
        cv::Point2f point_2d = list_points[i];

        // Draw Selected points
        cv::circle(image, point_2d, radius, color, -1, lineType );
    }
}

// Draw an arrow into the image
void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
    //Draw the principle line
    cv::line(image, p, q, color, thickness, line_type, shift);
    const double PI = CV_PI;
    //compute the angle alpha
    double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
    //compute the coordinates of the first segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
    //Draw the first segment
    cv::line(image, p, q, color, thickness, line_type, shift);
    //compute the coordinates of the second segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
    //Draw the second segment
    cv::line(image, p, q, color, thickness, line_type, shift);
}

// Draw the 3D coordinate axes
void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d)
{
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0,255,0);
    cv::Scalar blue(255,0,0);
    cv::Scalar black(0,0,0);

    cv::Point2i origin = list_points2d[0];
    cv::Point2i pointX = list_points2d[1];
    cv::Point2i pointY = list_points2d[2];
    cv::Point2i pointZ = list_points2d[3];

    drawArrow(image, origin, pointX, red, 9, 2);
    drawArrow(image, origin, pointY, green, 9, 2);
    drawArrow(image, origin, pointZ, blue, 9, 2);
    cv::circle(image, origin, radius/2, black, -1, lineType );
}

// Draw the object mesh

// Computes the norm of the translation error
double get_translation_error(const cv::Mat &t_true, const cv::Mat &t)
{
    return cv::norm( t_true - t );
}

// Computes the norm of the rotation error
double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R)
{
    cv::Mat error_vec, error_mat;
    error_mat = -R_true * R.t();
    cv::Rodrigues(error_mat, error_vec);

    return cv::norm(error_vec);
}

// Converts a given Rotation Matrix to Euler angles
// Convention used is Y-Z-X Tait-Bryan angles
// Reference code implementation:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI/2;
        heading = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI/2;
        heading = atan2(m02,m22);
    }
    else
    {
        bank = atan2(-m12,m11);
        attitude = asin(m10);
        heading = atan2(-m20,m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat euler2rot(const cv::Mat & euler)
{
    cv::Mat rotationMatrix(3,3,CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;

    rotationMatrix.at<double>(0,0) = m00;
    rotationMatrix.at<double>(0,1) = m01;
    rotationMatrix.at<double>(0,2) = m02;
    rotationMatrix.at<double>(1,0) = m10;
    rotationMatrix.at<double>(1,1) = m11;
    rotationMatrix.at<double>(1,2) = m12;
    rotationMatrix.at<double>(2,0) = m20;
    rotationMatrix.at<double>(2,1) = m21;
    rotationMatrix.at<double>(2,2) = m22;

    return rotationMatrix;
}

// Converts a given string to an integer
int StringToInt ( const std::string &Text )
{
    std::istringstream ss(Text);
    int result;
    return ss >> result ? result : 0;
}
double StringTodouble ( const std::string &Text )
{
    std::istringstream ss(Text);
    double result;
    return ss >> result ? result : 0;
}

// Converts a given float to a string
std::string FloatToString ( float Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

// Converts a given integer to a string
std::string IntToString ( int Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

void createFeatures(const std::string &featureName, int numKeypoints, cv::Ptr<cv::Feature2D> &detector, cv::Ptr<cv::Feature2D> &descriptor)
{
    if (featureName == "ORB")
    {
        detector = cv::ORB::create(numKeypoints);
        descriptor = cv::ORB::create(numKeypoints);
    }
    else if (featureName == "KAZE")
    {
        detector = cv::KAZE::create();
        descriptor = cv::KAZE::create();
    }
    else if (featureName == "AKAZE")
    {
        detector = cv::AKAZE::create();
        descriptor = cv::AKAZE::create();
    }
    else if (featureName == "BRISK")
    {
        detector = cv::BRISK::create();
        descriptor = cv::BRISK::create();
    }
    else if (featureName == "SIFT")
    {
#if defined (OPENCV_ENABLE_NONFREE) && defined (HAVE_OPENCV_XFEATURES2D)
        detector = cv::xfeatures2d::SIFT::create();
        descriptor = cv::xfeatures2d::SIFT::create();
#else
        std::cout << "xfeatures2d module is not available or nonfree is not enabled." << std::endl;
        std::cout << "Default to ORB." << std::endl;
        detector = cv::ORB::create(numKeypoints);
        descriptor = cv::ORB::create(numKeypoints);
#endif
    }
    else if (featureName == "SURF")
    {
#if defined (OPENCV_ENABLE_NONFREE) && defined (HAVE_OPENCV_XFEATURES2D)
        detector = cv::xfeatures2d::SURF::create(100, 4, 3, true);   //extended=true
        descriptor = cv::xfeatures2d::SURF::create(100, 4, 3, true); //extended=true
#else
        std::cout << "xfeatures2d module is not available or nonfree is not enabled." << std::endl;
        std::cout << "Default to ORB." << std::endl;
        detector = cv::ORB::create(numKeypoints);
        descriptor = cv::ORB::create(numKeypoints);
#endif
    }
    else if (featureName == "BINBOOST")
    {
#if defined (HAVE_OPENCV_XFEATURES2D)
        detector = cv::KAZE::create();
        descriptor = cv::xfeatures2d::BoostDesc::create();
#else
        std::cout << "xfeatures2d module is not available." << std::endl;
        std::cout << "Default to ORB." << std::endl;
        detector = cv::ORB::create(numKeypoints);
        descriptor = cv::ORB::create(numKeypoints);
#endif
    }
    else if (featureName == "VGG")
    {
#if defined (HAVE_OPENCV_XFEATURES2D)
        detector = cv::KAZE::create();
        descriptor = cv::xfeatures2d::VGG::create();
#else
        std::cout << "xfeatures2d module is not available." << std::endl;
        std::cout << "Default to ORB." << std::endl;
        detector = cv::ORB::create(numKeypoints);
        descriptor = cv::ORB::create(numKeypoints);
#endif
    }
}

cv::Ptr<cv::DescriptorMatcher> createMatcher(const std::string &featureName, bool useFLANN)
{
    if (featureName == "ORB" || featureName == "BRISK" || featureName == "AKAZE" || featureName == "BINBOOST")
    {
        if (useFLANN)
        {
            cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
            cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
            return cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
        }
        else
        {
            return cv::DescriptorMatcher::create("BruteForce-Hamming");
        }

    }
    else
    {
        if (useFLANN)
        {
            return cv::DescriptorMatcher::create("FlannBased");
        }
        else
        {
            return cv::DescriptorMatcher::create("BruteForce");
        }
    }
}
