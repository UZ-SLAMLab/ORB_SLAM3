#include "nanoflann.hpp"
#include <opencv2/core.hpp>
#include "DBoW3.h"
#include "timers.h"
#include <memory>
#include <opencv2/flann.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
#include <cstdlib>
#include "DescManip.h"
using namespace DBoW3;
using namespace std;
using namespace std;


std::vector< cv::Mat  >  loadFeatures( std::vector<string> path_to_images,string descriptor="") throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")   fdetector=cv::ORB::create(2000);

    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
#ifdef OPENCV_VERSION_3
    else if (descriptor=="akaze") fdetector=cv::AKAZE::create();
#endif
#ifdef USE_CONTRIB
    else if(descriptor=="surf" )  fdetector=cv::xfeatures2d::SURF::create(400, 4, 2, false);
#endif

    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    vector<cv::Mat>    features;


    cout << "Extracting   features..." << endl;
    for(size_t i = 0; i < path_to_images.size(); ++i)
    {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cout<<"reading image: "<<path_to_images[i]<<endl;
        cv::Mat image = cv::imread(path_to_images[i], 0);
        if(image.empty())throw std::runtime_error("Could not open image"+path_to_images[i]);
        cout<<"extracting features"<<endl;
        fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        features.push_back(descriptors);
        cout<<"done detecting features"<<endl;
    }
    return features;
}
namespace DBoW3{
class FastSearch{
public:

    //get features
    static cv::Mat features(DBoW3::Vocabulary &voc){
        uint desc_size=DBoW3::DescManip::getDescSizeBytes(voc.m_words[0]->descriptor);
        cv::Mat feat(voc.m_words.size(),desc_size,CV_8UC1);

        for(int i=0;i< voc.m_words.size();i++){
            memcpy(feat.ptr<uchar>(i),voc.m_words[i]->descriptor.ptr<uchar>(0),desc_size);
            assert(i==voc.m_words[i]->word_id);
        }
        return feat;
    }
};
}


int main(int argc,char **argv){
    DBoW3::Vocabulary voc;
    voc.load(argv[1]);
    cout<<"loaded"<<endl;
    DBoW3::BowVector vv;
    auto features=loadFeatures({argv[2]},"orb");

    {ScopeTimer t("dbow");
            voc.transform(features[0],vv);
    }
    cout<<vv.begin()->first<<" "<<vv.begin()->second<<endl;
    cout<<vv.rbegin()->first<<" "<<vv.rbegin()->second<<endl;

    auto voc_feat=FastSearch::features(voc);
    cv::flann::HierarchicalClusteringIndexParams indexParams(10,cvflann::FLANN_CENTERS_RANDOM,4,1) ;
//    cv::flann::LshIndexParams indexParams(20,10,2);
    ScopedTimerEvents timer("flann");

    cv::flann::Index kdtree(voc_feat, indexParams,cvflann::FLANN_DIST_HAMMING);
    timer.add("build index");



    cout << "Performing single search to find closest data point to mean:" << endl;

    cv::Mat indices;//(numQueries, k, CV_32S);
    cv::Mat dists;//(numQueries, k, CV_32F);

    {
        ScopedTimerEvents timer("search");
        // Invoke the function
        kdtree.knnSearch(features[0], indices, dists, 1, cv::flann::SearchParams(1));
    }
    cout<<"res="<<indices.at<int>(0,0)<<" "<<indices.at<int>(indices.rows-1,0)<<endl;
    //    cout << indices.rows << "\t" << indices.cols << endl;
    //    cout << dists.rows << "\t" << dists.cols << endl;

    //    // Print batch results
    //    cout << "Output::"<< endl;
    //    for(int row = 0 ; row < indices.rows ; row++){
    //        cout << "(index,dist):";
    //        for(int col = 0 ; col < indices.cols ; col++){
    //            cout << "(" << indices.at<int>(row,col) << "," << dists.at<float>(row,col) << ")" << "\t";
    //        }
    //        cout << endl;
    //    }





}
