//Second step,creates the vocabulary from the set of features. It can be slow
#include <iostream>
#include <vector>

// DBoW3
#include "DBoW3.h"

// OpenCV
#include <opencv2/core/core.hpp>
using namespace DBoW3;
using namespace std;

//command line parser
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
vector<cv::Mat> readFeaturesFromFile(string filename){
vector<cv::Mat> features;
    //test it is not created
    std::ifstream ifile(filename);
    if (!ifile.is_open()){cerr<<"could not open input file"<<endl;exit(0);}
    uint32_t size;
    ifile.read((char*)&size,sizeof(size));
    features.resize(size);
    for(size_t i=0;i<size;i++){

        uint32_t cols,rows,type;
        ifile.read( (char*)&cols,sizeof(cols));
        ifile.read( (char*)&rows,sizeof(rows));
        ifile.read( (char*)&type,sizeof(type));
        features[i].create(rows,cols,type);
        ifile.read( (char*)features[i].ptr<uchar>(0),features[i].total()*features[i].elemSize());
    }
    return features;
}

// ----------------------------------------------------------------------------

int main(int argc,char **argv)
{

    try{
        CmdLineParser cml(argc,argv);
        if (cml["-h"] || argc!=3){
            cerr<<"Usage:  features output_voc.yml[.gz]"<<endl;
            return -1;
        }


        auto features=readFeaturesFromFile(argv[1]);

        const int k = 9;
        const int L = 3;
        const WeightingType weight = TF_IDF;
        const ScoringType score = L1_NORM;
        DBoW3::Vocabulary voc (k, L, weight, score);

        cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
        voc.create(features);
        cerr<<"Saving "<<argv[2]<<endl;
        voc.save(argv[2]);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

    return 0;
}
