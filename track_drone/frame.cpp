//
// Created by firas on 9.4.2020.
//
//#include <opencv2/xfeatures2d/nonfree.hpp>
#include "RobustMatcher.h"
#include "Utils.h"
//#include "mkl.h"

#include "frame.h"
//#include <armadillo>
//#define M 1
//#define N 3
//#define LDA M
//#define LDU M
//#define LDVT N
frame::frame() : n_correspondences_(0),list_keypoints_(0), list_points2d_(0),list_lines_(0),  list_points3d_(0)
{
}
frame::~frame()
{
    // TODO Auto-generated destructor stub
}
void frame::loadframe(cv::Mat &current_frame,int &numKeyPoints2,float &ratiotest,cv::Mat &cameraMatrix) {
//    frame_ = current_frame;
//    rmatcher_

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(numKeyPoints2);
    cv::Ptr<cv::FeatureDetector> descriptor = cv::ORB::create(numKeyPoints2);
//    cv::Ptr<cv::FeatureDetector> detector2 = cv::xfeatures2d::SIFT::create();
//    cv::Ptr<cv::FeatureDetector> descriptor2 = cv::xfeatures2d::SIFT::create();
//    cv::Ptr<cv::FeatureDetector> sift = cv::SIFT::create(8000);
    rmatcher_.setFeatureDetector(detector);                                // set feature detector
    rmatcher_.setDescriptorExtractor(descriptor);                          // set descriptor extractor
    rmatcher_.setDescriptorMatcher(createMatcher("ORB", true));     // set matcher
    rmatcher_.setRatio(ratiotest);
//    cv::Mat mask;
//    sift->detectAndCompute(current_frame,mask,list_keypoints_,descriptors_);
//    std::cout<<descriptors_.size()<<std::endl;
    rmatcher_.computeKeyPoints(current_frame,list_keypoints_);
    rmatcher_.calcdesriptorsperframe(current_frame,list_keypoints_,descriptors_);
    std::vector<cv::Point2f> temppoint2d;
    std::vector<cv::Point3f> templines;
    for (auto & list_keypoint : list_keypoints_) {
        cv::Point2f point2d = list_keypoint.pt; // 2D point from the scene
        temppoint2d.push_back(point2d);         // add 2D point
        cv::Mat pt = (cv::Mat_<float>(3,1)<<point2d.x, point2d.y, 1);
        cv::Mat invK=cameraMatrix.inv();
        cv::Mat templine= invK*pt;
        templine=templine/norm(templine);
        cv::Point3f v(templine) ;
        templines.push_back(v);

    }
//    std::cout<<templines<<std::endl;
    list_lines_=templines;
//    std::cout<<list_lines_<<std::endl;

    list_points2d_=temppoint2d;
//    std::vector<cv::Point3d> lines_double(list_lines_.begin(),list_lines_.end());
//    calculate_line_matrix(lines_double,list_lines_4coreset_);
}
//cv::Mat frame::Null_line3(cv::Point3f lin){
//    cv::Mat  A(lin),w, u, vt;
//    cv::SVD::compute(A, w, u, vt,cv::SVD::FULL_UV);
//    flip(u,u ,1);
//    return u.t();
//}
//cv::Mat frame::Null_line(cv::Point3d lin){
//    cv::Mat  A(lin),w, u, vt;
//    cv::SVD::compute(A, w, u, vt,cv::SVD::FULL_UV);
//    u=u.t();
////    flip(u,u ,0);
//    cv::Mat uu;
//    uu.push_back(u.row(1));
//    uu.push_back(u.row(2));
//    uu.push_back(u.row(0));
//    return uu;
//}
//void frame::prep_for_coreset(){
//    int counter=0;
////    std::cout<<list_lines_<<std::endl;
//
//    for(auto line: list_lines_){
//        cv::Mat lineMatrix;
//        cv::Mat templinematrix= Null_line(cv::Point3d(line));
//        templinematrix(cv::Range(0, templinematrix.rows - 1), cv::Range(0, templinematrix.cols)).copyTo(lineMatrix);
//        list_lines_4coreset_.push_back(lineMatrix);
//        counter++;
//    }
//}
//cv::Mat frame::Null_line3(cv::Point3d lin){
////    cv::Mat  A(lin),w, u, vt,new_v;
////    auto start1 = std::chrono::steady_clock::now();
////    double vtt[LDVT*N];
////    double vttt[LDVT*N];
//////    fi_SVD(lin,vtt);
////    fi_SVD(A,vttt);
////    double a[3]={lin.x,lin.y,lin.z};
//////    double *a=&a_vector[0];
////    MKL_INT m = M, n = N, lda = LDA, ldu = LDU, ldvt = LDVT, info, lwork;
////    double wkopt;
////    double* work;
////    /* iwork dimension should be at least 8*min(m,n) */
////    MKL_INT iwork[8*N];
////    double s[N], u2[LDU*M];
////    /* Executable statements */
////    printf( " DGESDD Example Program Results\n" );
////    /* Query and allocate the optimal workspace */
////    lwork = -1;
////    dgesdd( "A", &m, &n, a, &lda, s, u2, &ldu, vtt, &ldvt, &wkopt,
////            &lwork, iwork, &info );
////    lwork = (int)wkopt;
////    work = (double*)malloc( lwork*sizeof(double) );
////    /* Compute SVD */
//////    auto start2 = std::chrono::steady_clock::now();
////    dgesdd( "A", &m, &n, a, &lda, s, u2, &ldu, vtt, &ldvt, work,
////            &lwork, iwork, &info );
////
////    auto end1 = std::chrono::steady_clock::now();
////    std::chrono::duration<double> elapsed_seconds1 = end1-start1;
////    std::cout<<elapsed_seconds1.count()<<endl;
////    print_matrix( "Right singular vectors (stored rowwise)", 3, 3, vtt, 3 );
////    print_matrix( "Right singular vectors (stored rowwise)", 3, 3, vttt, 3 );
////    int i, j;
////    cv::Mat uu(2,3,CV_64F);
////    for( i =1; i < 3; i++ ) {
////        cv::Mat Row;
////        for (j = 0; j < 3; j++) {
////            Row.push_back(vtt[i + j * 3]);
////
////            cout << Row << endl;
////        }
////        Row=Row.t();
////        Row.copyTo(uu.row(i-1));
////    }
////    cout<<uu<<endl;
////    free( (void*)work );
//
//    cv::Mat  A(lin),w, vt , uu;
////    cv::Mat At=A.t();
////    arma::mat arma_A( reinterpret_cast<double*>(At.data), A.rows, A.cols );
////    arma::mat U, Var;
////    arma::vec S;
////    arma::svd(U, S, Var, arma_A,"dc");
////    cv::Mat u(  A.rows,A.rows, CV_64F, U.memptr() );
////    cv::Mat uu;
////    uu.push_back(u.row(1));
////    uu.push_back(u.row(2));
////    uu.push_back(u.row(0));
//    return uu;
////    std::cout<<uu<<std::endl;
//
//    return uu;
//}
//void frame::prep_for_coreset3(){
//    int counter=0;
////    std::cout<<list_lines_<<std::endl;
//
//    for(auto line: list_lines_){
//        cv::Mat lineMatrix;
//        cv::Mat templinematrix= Null_line3(line);
////        std::cout<<templinematrix<<std::endl;
////        templinematrix(cv::Range(0, templinematrix.rows), cv::Range(0, templinematrix.cols)).copyTo(lineMatrix);
//        templinematrix(cv::Range(0, templinematrix.rows - 1), cv::Range(0, templinematrix.cols)).copyTo(lineMatrix);
////        std::cout<<lineMatrix<<endl;
//        list_lines_4coreset_.push_back(lineMatrix);
////        std::cout<<list_lines_4coreset[0]<<endl;
//        counter++;
//    }
//}
//cv::Mat frame::Null_space(cv::Point3d lin){
//    cv::Mat  A(lin),w, vt;
//    double vtt[LDVT*N];
//    fi_SVD(A,vtt);
//    cv::Mat u(  A.rows,A.rows, CV_64F, vtt );
//    cv::Mat uu;
//    uu.push_back(u.row(1));
//    uu.push_back(u.row(2));
//    uu.push_back(u.row(0));
//    return uu;
//}
//void frame::calculate_line_matrix(std::vector<cv::Point3d> list_lines ,std::vector<cv::Mat> &list_lines_4coreset){
//    for(auto line: list_lines){
//        cv::Mat lineMatrix;
////        calculate null space of diretion vector
//        cv::Mat templinematrix= Null_space(line);
////        templinematrix(cv::Range(0, templinematrix.rows - 1), cv::Range(0, templinematrix.cols)).copyTo(lineMatrix);
//        lineMatrix=templinematrix(cv::Range(0, templinematrix.rows - 1), cv::Range(0, templinematrix.cols)).clone();
//        list_lines_4coreset.push_back(lineMatrix);
//    }
//}
//void frame::fi_SVD(cv::Mat A_Mat, double *vt){
//    std::vector<double> a_vector=A_Mat;
//    double *a=&a_vector[0];
//
//    MKL_INT m = M, n = N, lda = LDA, ldu = LDU, ldvt = LDVT, info, lwork;
//    double wkopt;
//    double* work;
//    /* iwork dimension should be at least 8*min(m,n) */
//    MKL_INT iwork[8*N];
//    double s[N], u[LDU*M];
//    /* Executable statements */
////    printf( " DGESDD Example Program Results\n" );
//    /* Query and allocate the optimal workspace */
//    lwork = -1;
//    dgesdd( "A", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, &wkopt,
//            &lwork, iwork, &info );
//    lwork = (int)wkopt;
//    work = (double*)malloc( lwork*sizeof(double) );
//    /* Compute SVD */
////    auto start2 = std::chrono::steady_clock::now();
//    dgesdd( "A", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, work,
//            &lwork, iwork, &info );
//    free(work);
////    auto end2 = std::chrono::steady_clock::now();
////    std::chrono::duration<double> elapsed_seconds2 = end2-start2;
////    std::cout<<"kakaka"<<elapsed_seconds2.count()<<endl;
////    print_matrix( "Singular values", 1, n, s, 1 );
////    /* Print left singular vectors */
////    print_matrix( "Left singular vectors (stored columnwise)", m, n, u, ldu );
////    /* Print right singular vectors */
////    print_matrix( "Right singular vectors (stored rowwise)", n, n, vt, ldvt );
////    cv::Mat A(nin, nin, CV_64F,vt);
////    A.copyTo(newMat);
////    cout<<A<<endl;
////    int bobo=0;
//}