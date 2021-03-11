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


#ifndef iSLAM_TORCH_HELPERS
#define iSLAM_TORCH_HELPERS

#include <torch/torch.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#if (CV_VERSION_MAJOR >= 4)
  #include<opencv2/imgcodecs/legacy/constants_c.h>
#endif

#include <string>

namespace ORB_SLAM3
{

std::string GetImageType( const cv::Mat& img, bool more_info = true );

void ShowImage( cv::Mat img, std::string title );

at::Tensor TransposeTensor( at::Tensor & tensor, 
                            c10::IntArrayRef dims = { 0, 3, 1, 2 } );

at::Tensor CVImgToTensor( cv::Mat & img, 
                          bool unsqueeze = false,
                          int unsqueeze_dim = 0 );

cv::Mat ToCvImage( at::Tensor & tensor );

} // namespace ORB_SLAM3

#endif // iSLAM_TORCH_HELPERS