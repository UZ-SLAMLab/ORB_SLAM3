#include <iostream>
#include <vector>

// DBoW3
#include "DBoW3.h"
#include "timers.h"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
using namespace DBoW3;
using namespace std;

//command line parser
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

vector< cv::Mat  >  loadFeatures( std::vector<string> path_to_images,string descriptor="") throw (std::exception){
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


// ----------------------------------------------------------------------------

int main(int argc,char **argv)
{

    try{
        CmdLineParser cml(argc,argv);
        if (cml["-h"] || argc!=3){
            cerr<<"Usage:  invoc.yml image1 "<<endl;
            return -1;
        }
        DBoW3::Vocabulary voc;
        voc.load(argv[1]);
        cout<<"loaded"<<endl;
        auto features=loadFeatures({argv[2]},"orb");
        cout<<"size="<<features[0].rows<<endl;
        DBoW3::BowVector vv;
        cout<<"size="<<features[0].rows<<endl;
        auto t_start=std::chrono::high_resolution_clock::now();
        for(int i=0;i<1000;i++)
            voc.transform(features[0],vv);
        auto t_end=std::chrono::high_resolution_clock::now();
        cout<<"time="<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(t_end-t_start).count())/1e6<<" ns"<<endl;

        cout<<vv.begin()->first<<" "<<vv.begin()->second<<endl;
        cout<<vv.rbegin()->first<<" "<<vv.rbegin()->second<<endl;

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

    return 0;
}

