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

int main(int argc,char **argv)
{

    try{
        CmdLineParser cml(argc,argv);
        if (cml["-h"] || argc!=3){
            cerr<<"Usage:  in.yml out.dbow"<<endl;
            return -1;
        }
        DBoW3::Vocabulary voc;
        voc.load(argv[1]);
        cout<<"loaded"<<endl;
        voc.save(argv[2]);
//        DBoW3::Vocabulary voc2;
//        voc2.load(argv[2]);
//        cout<<"loaded"<<endl;
//        voc2.save(string(argv[2])+"-copy",false);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

    return 0;
}
