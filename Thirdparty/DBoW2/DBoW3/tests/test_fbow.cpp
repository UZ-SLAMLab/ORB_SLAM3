#include "nanoflann.hpp"
#include <opencv2/core.hpp>
#include "DBoW3.h"
#include "timers.h"
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
#include <cstdlib>
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



namespace DBoW3 {
class FastSearch{
public:
     ~FastSearch(){if (_data) delete _data;}

    int getNodePos( Vocabulary::Node & node){

    }
    struct node_info{
            uint32_t id_or_childblock; //if id ,msb is 1.

        float weight;

        inline bool isleaf()const{return ( id_or_childblock& 0x80000000);}
        inline uint32_t getChildBlock()const{return ( id_or_childblock);}
        inline uint32_t getId()const{return ( id_or_childblock&0x7FFFFFFF);}
    };

    std::map<uint32_t,set<uint32_t > > parent_children;
    void create(Vocabulary &voc){
        if(voc.getDescritorType()==CV_8UC1) _aligment=8;
        else _aligment=16;



        //consider possible aligment of each descriptor adding offsets at the end
        _desc_size_bytes=voc.getDescritorSize();
        _desc_size_bytes_al=_desc_size_bytes/_aligment;
        if(_desc_size_bytes%_aligment!=0) _desc_size_bytes_al++;
        _desc_size_bytes=_desc_size_bytes_al*_aligment;


        int foffnbytes_alg=sizeof(uint32_t)/_aligment;
        if(sizeof(uint32_t)%_aligment!=0) foffnbytes_alg++;
        _feature_off_start=foffnbytes_alg*_aligment;
        _child_off_start=_feature_off_start+voc.m_k*_desc_size_bytes ;//where do children information start from the start of the block


        //block: nvalid|f0 f1 .. fn|ni0 ni1 ..nin
        _block_size_bytes=_feature_off_start+ voc.m_k * (_desc_size_bytes + sizeof(node_info));
        _block_size_bytes_al=_block_size_bytes/_aligment;
        if (_block_size_bytes%_aligment!=0) _block_size_bytes_al++;
        _block_size_bytes=_block_size_bytes_al*_aligment;


        _desc_type=CV_8UC1;
        _desc_size=32;


        _m_k=voc.m_k;
        //start to work filling blocks
        cout<<"_aligment="<<_aligment<<endl;
        cout<<"_nblocks="<<_nblocks<<endl;
        cout<<"_desc_size_bytes="<<_desc_size_bytes<<endl;
        cout<<"_desc_size_bytes_al="<<_desc_size_bytes_al<<endl;
        cout<<"_block_size_bytes="<<_block_size_bytes<<endl;
        cout<<"_block_size_bytes_al="<<_block_size_bytes_al<<endl;
        cout<<"_child_off_start="<<_child_off_start<<endl;
        cout<<"voc.size()="<<voc.size()<<endl;
        cout<<"voc.m_k="<<voc.m_k<<endl;
        cout<<"voc.m_L="<<voc.m_L<<endl;

        cerr<<"creating index"<<endl;
        std::map<uint32_t,uint32_t> nid_vpos;
        for(size_t i=0;i<voc.m_nodes.size();i++){
            auto &n=voc.m_nodes[i];
            if (n.id!=0) {
                parent_children[n.parent].insert(n.id);
                nid_vpos[n.id]=i;
            }
        }
        for(auto &p:parent_children){
            //assert(p.size()==voc.m_k);
        }
        //now, we know the number of blocks

        //how many blocks (oversampled)
        _nblocks= parent_children.size();
        cout<<_nblocks<<endl;
        //give memory
        _total_size=_block_size_bytes*_nblocks;
        _data=(char*)aligned_alloc(_aligment,_total_size);
        memset(_data,0xffff,_total_size);



        cerr<<"creating done _total_size="<<_total_size/(1024*1024)<<" "<<_total_size<<endl;


        cout<<parent_children.begin()->first<<endl;
        std::map<uint32_t,uint32_t> block_offset;
        uint32_t currblock=0;//expressed in blocks
        uint32_t descsize=voc.getDescritorSize();
        for(const auto &Block:parent_children)
         {
            block_offset[Block.first]=currblock;
            assert( !(currblock & 0x80000000));//32 bits 100000000...0.check msb is not set
            uint64_t block_offset_bytes=currblock*_block_size_bytes;
            int idx=0;
             *reinterpret_cast<uint32_t*>(_data+block_offset_bytes)=Block.second.size();
            for(const auto &c:Block.second){
                const auto &node=voc.m_nodes[nid_vpos[c]];
                memcpy(_data+block_offset_bytes+_feature_off_start+idx*_desc_size_bytes,node.descriptor.ptr<char>(0),descsize);
                assert( block_offset_bytes+idx*_desc_size_bytes +descsize < _total_size );
                //now, the offset to the children block//unkonwn yet
                idx++;
            }
            currblock++;
        }
        currblock=0;
        //print sons of node 6

        //now, we can write the offsets
        for(const auto &Block:parent_children)
         {

            int idx=0;
            uint64_t block_offset_bytes=currblock*_block_size_bytes;
            for(const auto &c:Block.second){
                const auto &node=voc.m_nodes[nid_vpos[c]];
                node_info *ptr_child=(node_info*)(_data+block_offset_bytes+_child_off_start+sizeof(node_info)*idx);

                if (!node.isLeaf()) {
                    assert(block_offset.count(node.id));
                    ptr_child->id_or_childblock=block_offset[node.id];//childblock
                }
                else{
                    //set the node id (ensure msb is set)
                    assert(!(node.id & 0x80000000));//check
                    ptr_child->id_or_childblock=node.word_id;
                    ptr_child->id_or_childblock|=0x80000000;//set the msb to one to distinguish from offset
                    //now,set the weight too
                    ptr_child->weight=node.weight;
                }
                //now, the offset to the children block//unkonwn yet
                idx++;
            }
            currblock++;
        }
        cout<<"nblocks used="<<currblock<<" reserved="<<_nblocks<<endl;
    }

    node_info* getBestOfBlock(int block,const cv::Mat &desc){
        uint64_t block_start=block*_block_size_bytes;
        uint32_t mind=std::numeric_limits<uint32_t>::max();
        uint32_t n=*reinterpret_cast<int*>(_data+block_start);
        const uchar *dptr=desc.ptr<uchar>(0);
        int bestIdx=0;
        for(int i=0;i<n;i++)
        {
            uchar  *ptr=(uchar*)(_data+block_start+_feature_off_start+_desc_size_bytes*i);
            uint32_t d=0;
            for(int j=0;j<_desc_size_bytes;j++)   d+= __builtin_popcount( dptr[j]^ptr[j]);
            if (d<mind){
                mind=d;
                bestIdx=i;
            }
        }
        return (node_info*)(_data+ block_start+_child_off_start+sizeof(node_info)*bestIdx);
    }

    inline bool getNodeInfo(int block,int id){
        return (node_info*)(_data+ (block*_block_size_bytes)+_child_off_start+sizeof(node_info)*id);
    }


    void transform(const cv::Mat &desc,DBoW3::BowVector &v){

        for(int i=0;i<desc.rows;i++){

            //
            bool done=false;
            int block=0;
            float weight;int wid;
            while(!done){
                node_info *ni;//=getBestOfBlock(block,desc.row(i));
{
                    uint64_t block_start=block*_block_size_bytes;
                    uint32_t mind=std::numeric_limits<uint32_t>::max();
                    uint32_t n=*reinterpret_cast<uint32_t*>(_data+block_start);
                    const uint64_t *dptr=desc.ptr<uint64_t>(i);
                    int bestIdx=0;
                    int n4=n/4;
                    uint64_t _toff=block_start+_feature_off_start;
                    int i=0;
                    for(i=0;i<n4;i+=4)
                    {
                        uint64_t  *ptr=(uint64_t*)(_data+_toff+_desc_size_bytes*i);
                        uint32_t d=__builtin_popcountl(dptr[0]^ptr[0])+ __builtin_popcountl(dptr[1]^ptr[1])+__builtin_popcountl(dptr[2]^ptr[2])+__builtin_popcountl(dptr[3]^ptr[3]);
                        uint64_t  *ptr_2=(uint64_t*)(_data+_toff+_desc_size_bytes*(i+1));
                        uint32_t d2=__builtin_popcountl(dptr[0]^ptr_2[0])+ __builtin_popcountl(dptr[1]^ptr_2[1])+__builtin_popcountl(dptr[2]^ptr_2[2])+__builtin_popcountl(dptr[3]^ptr_2[3]);
                        uint64_t  *ptr_3=(uint64_t*)(_data+_toff+_desc_size_bytes*(i+2));
                        uint32_t d3=__builtin_popcountl(dptr[0]^ptr_3[0])+ __builtin_popcountl(dptr[1]^ptr_3[1])+__builtin_popcountl(dptr[2]^ptr_3[2])+__builtin_popcountl(dptr[3]^ptr_3[3]);
                        uint64_t  *ptr_4=(uint64_t*)(_data+_toff+_desc_size_bytes*(i+3));
                        uint32_t d4=__builtin_popcountl(dptr[0]^ptr_4[0])+ __builtin_popcountl(dptr[1]^ptr_4[1])+__builtin_popcountl(dptr[2]^ptr_4[2])+__builtin_popcountl(dptr[3]^ptr_4[3]);
                        if (d<mind){
                            mind=d;
                            bestIdx=i;
                        }
                        if (d2<mind){
                            mind=d2;
                            bestIdx=i+1;
                        }
                        if (d3<mind){
                            mind=d3;
                            bestIdx=i+2;
                        }
                        if (d4<mind){
                            mind=d4;
                            bestIdx=i+3;
                        }

                    }
                for( ;i<n;i++)
                {
                    uint64_t  *ptr=(uint64_t*)(_data+_toff+_desc_size_bytes*i);
                    uint32_t d=__builtin_popcountl(dptr[0]^ptr[0])+ __builtin_popcountl(dptr[1]^ptr[1])+__builtin_popcountl(dptr[2]^ptr[2])+__builtin_popcountl(dptr[3]^ptr[3]);
                    if (d<mind){
                        mind=d;
                        bestIdx=i;
                    }
                }
                ni= (node_info*)(_data+ block_start+_child_off_start+sizeof(node_info)*bestIdx);

                }
                if (ni->isleaf()){
                    wid=ni->getId();
                    weight=ni->weight;
                    done=true;
                }
                else//go to children block
                    block=ni->getChildBlock();
            }
            v.addWeight(wid,weight);
        }
    }

    uint32_t _aligment,_nblocks;
    uint64_t _desc_size_bytes;//size of the descriptor(includding padding)
    uint64_t _desc_size_bytes_al;
    uint64_t _block_size_bytes;//size of a block
    uint64_t _block_size_bytes_al;
    uint64_t _feature_off_start;
    uint64_t _child_off_start;//into the block,where the children offset part starts
    uint64_t _total_size;
    int32_t _desc_type,_desc_size;//original desc type and size
    uint32_t _m_k;//number of children per node
    char *_data=0;
    string _desc_name;

    void saveToFile(const std::string &filepath)throw(std::exception){
        std::ofstream str(filepath);
        if (!str) throw std::runtime_error("Vocabulary::saveToFile could not open:"+filepath);
        //magic number
        uint64_t sig=55824123;
        str.write((char*)&sig,sizeof(sig));

        _desc_name="orb";
        size_t str_s=_desc_name.size();
        str.write((char*)&str_s,sizeof(str_s));
        str.write(_desc_name.c_str(),_desc_name.size());

        str.write((char*)&_aligment,sizeof(_aligment));
        str.write((char*)&_nblocks,sizeof(_nblocks));
        str.write((char*)&_desc_size_bytes,sizeof(_desc_size_bytes));
        str.write((char*)&_desc_size_bytes_al,sizeof(_desc_size_bytes_al));
        str.write((char*)&_block_size_bytes,sizeof(_block_size_bytes));
        str.write((char*)&_block_size_bytes_al,sizeof(_block_size_bytes_al));
        str.write((char*)&_feature_off_start,sizeof(_feature_off_start));
        str.write((char*)&_child_off_start,sizeof(_child_off_start));
        str.write((char*)&_total_size,sizeof(_total_size));
        str.write((char*)&_m_k,sizeof(_m_k));
        str.write((char*)&_desc_type,sizeof(_desc_type));        
        str.write((char*)&_desc_size,sizeof(_desc_size));
        str.write(_data,_total_size);
        cout<<"save:"<<int(_data[0])<<" "<<int(_data[1])<<" "<<int(_data[2])<<" "<<int(_data[3])<<endl;
        cout<<"save:"<<*reinterpret_cast<uint32_t*>(_data)<<endl;
    }

};
}


int main(int argc,char **argv){
    if (argc!=4){cerr<<"Usage voc.dbo3 image out.fbow"<<endl;return -1;}
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
    FastSearch fs;
    fs.create(voc);
    DBoW3::BowVector vv2;
    {ScopeTimer t("new");
        fs.transform(features[0],vv2);
    }
    fs.saveToFile(argv[3]);
    cout<<vv2.begin()->first<<" "<<vv2.begin()->second<<endl;
    cout<<vv2.rbegin()->first<<" "<<vv2.rbegin()->second<<endl;
}
