// Copyright 2019 srabiee@cs.utexas.edu
// College of Information and Computer Sciences,
// University of Texas at Austin
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "iv_slam_helpers/torch_helpers.h"

namespace ORB_SLAM3 {

std::string GetImageType( const cv::Mat& img, bool more_info )  
{
    int type = img.type();
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    uchar depth = type & CV_MAT_DEPTH_MASK;
    std::string r;

    switch( depth ) 
    {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += chans + '0';
   
    if( more_info )
    {
        std::cout << "Depth: " << img.depth() << " Channels: " << img.channels() << std::endl;
    }

    return r;
}

void ShowImage( cv::Mat img, std::string title ) 
{
    std::string image_type = GetImageType(img);
    cv::namedWindow( title + " type:" + image_type, cv::WINDOW_NORMAL ); 
    cv::imshow( title, img );
    cv::waitKey(0);

    return;
}

at::Tensor TransposeTensor( at::Tensor & tensor, 
                            c10::IntArrayRef dims ) 
{
    tensor = tensor.permute(dims);

    return tensor;
}

at::Tensor CVImgToTensor( cv::Mat & img, 
                          bool unsqueeze,
                          int unsqueeze_dim ) 
{
    at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kFloat);    // TODO - at::kByte instead?

    if( unsqueeze ) 
    {
        tensor_image.unsqueeze_( unsqueeze_dim );
    }

    return tensor_image;
}

cv::Mat ToCvImage( at::Tensor & tensor ) 
{
    int width = tensor.sizes()[2];
    int height = tensor.sizes()[3];

    try 
    {
        cv::Mat output_mat(cv::Size{ height, width }, CV_8UC1, tensor.data_ptr<uchar>()); // TODO - CV_32FC1, tensor.data_ptr<float>() instead?

        return output_mat;
    }

    catch( const c10::Error& e ) 
    {
        std::cout << "an error has occured : " << e.msg() << std::endl;
    }

    return cv::Mat(height, width, CV_8UC1); // TODO - CV_8UC1 instead?
}

} // namespace ORB_SLAM3