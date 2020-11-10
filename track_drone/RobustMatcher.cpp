/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "RobustMatcher.h"
#include <time.h>

#include <opencv2/features2d/features2d.hpp>

RobustMatcher::~RobustMatcher()
{
    // TODO Auto-generated destructor stub
}

void RobustMatcher::computeKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    detector_->detect(image, keypoints);
}

void RobustMatcher::computeDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    

    extractor_->compute(image, keypoints, descriptors);
    

   


}

int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
{
    int removed = 0;
    // for all matches
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        // if 2 NN has been identified
        if (matchIterator->size() > 1)
        {
            // check distance ratio
            if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio_)
            {
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        else
        { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}
int RobustMatcher::ratioTest2(std::vector<std::vector<cv::DMatch> > &matches)
{
    int removed = 0;
    // for all matches
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        // if 2 NN has been identified
        if (matchIterator->size() > 1)
        {
            // check distance ratio
            auto as=(*matchIterator)[0].distance;
            auto ass=(*matchIterator)[1].distance;
            auto ass1=(*matchIterator)[2].distance;
            auto ass2=(*matchIterator)[3].distance;
            auto ass3=(*matchIterator)[4].distance;
            if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio_ && (*matchIterator)[1].trainIdx==(*matchIterator)[1].queryIdx ||
            (*matchIterator)[0].trainIdx==(*matchIterator)[0].queryIdx)
            {
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        else
        { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}

void RobustMatcher::symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                                  const std::vector<std::vector<cv::DMatch> >& matches2,
                                  std::vector<cv::DMatch>& symMatches )
{
    // for all matches image 1 -> image 2
    for (std::vector<std::vector<cv::DMatch> >::const_iterator
         matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
    {
        // ignore deleted matches
        if (matchIterator1->empty() || matchIterator1->size() < 2)
            continue;

        // for all matches image 2 -> image 1
        for (std::vector<std::vector<cv::DMatch> >::const_iterator
             matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
        {
            // ignore deleted matches
            if (matchIterator2->empty() || matchIterator2->size() < 2)
                continue;

            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
                (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx)
            {
                // add symmetrical match
                symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,
                                     (*matchIterator1)[0].trainIdx, (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}

void RobustMatcher::robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                 std::vector<cv::KeyPoint>& keypoints_frame, const cv::Mat& descriptors_model,
                                 const std::vector<cv::KeyPoint>& keypoints_model)
{
    // 1a. Detection of the ORB features
    this->computeKeyPoints(frame, keypoints_frame);

    // 1b. Extraction of the ORB descriptors
    cv::Mat descriptors_frame;
    this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

    // 2. Match the two image descriptors
    std::vector<std::vector<cv::DMatch> > matches12, matches21;

    // 2a. From image 1 to image 2
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

    // 2b. From image 2 to image 1
    matcher_->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours

    // 3. Remove matches for which NN ratio is > than threshold
    // clean image 1 -> image 2 matches
    ratioTest(matches12);
    // clean image 2 -> image 1 matches
    ratioTest(matches21);

    // 4. Remove non-symmetrical matches
    symmetryTest(matches12, matches21, good_matches);

    if (!training_img_.empty() && !keypoints_model.empty())
    {
        cv::drawMatches(frame, keypoints_frame, training_img_, keypoints_model, good_matches, img_matching_);
    }
}


void RobustMatcher::calcdesriptorsperframe(const cv::Mat& frame,std::vector<cv::KeyPoint>& keypoints_frame,cv::Mat& descriptors_frame){
    // keypoints_frame.clear();
    descriptors_frame.release();
    // if(keypoints_frame.size()>0){
    //     std::cout<<"7777777777777777777777777777777777777"<<std::endl;
    //     std::cout<<keypoints_frame[0].size<<std::endl;
    //     std::cout<<keypoints_frame[0].angle<<std::endl;
    //     std::cout<<keypoints_frame[0].response<<std::endl;
    //     std::cout<<keypoints_frame[0].octave<<std::endl;
    //     std::cout<<keypoints_frame[0].class_id<<std::endl;
    //     std::cout<<"7777777777777777777777777777777777777"<<std::endl;
    // }
    // this->computeKeyPoints(frame, keypoints_frame);
        // 1b. Extraction of the ORB descriptors
    this->computeDescriptors(frame, keypoints_frame, descriptors_frame);
}

void RobustMatcher::fastRobustMatch2( std::vector<cv::DMatch>& good_matches,
                                     cv::Mat& descriptors_frame,
                                     const cv::Mat& descriptors_model)
{
    good_matches.clear();

    // 1a. Detection of the ORB features
    // this->computeKeyPoints(frame, keypoints_frame);
    // // std::cout<<"7777777777777777777777777777777777777"<<std::endl;
    // // std::cout<<keypoints_frame[0].size<<std::endl;
    // // std::cout<<keypoints_frame[0].angle<<std::endl;
    // // std::cout<<keypoints_frame[0].response<<std::endl;
    // // std::cout<<keypoints_frame[0].octave<<std::endl;
    // // std::cout<<keypoints_frame[0].class_id<<std::endl;
    // // std::cout<<"7777777777777777777777777777777777777"<<std::endl;

    // // 1b. Extraction of the ORB descriptors
    // cv::Mat descriptors_frame;
    // this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

    // 2. Match the two image descriptors
    std::vector<std::vector<cv::DMatch> > matches;
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2);

    // 3. Remove matches for which NN ratio is > than threshold
    ratioTest(matches);

    // 4. Fill good matches container
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }

    // if (!training_img_.empty() && !keypoints_model.empty())
    // {
    //     cv::drawMatches(frame, keypoints_frame, training_img_, keypoints_model, good_matches, img_matching_);
    // }
}
void RobustMatcher::fastRobustMatch3( std::vector<cv::DMatch>& good_matches,
                                     cv::Mat& descriptors_frame,
                                     const cv::Mat& descriptors_model)
{
    good_matches.clear();
    // 2. Match the two image descriptors
    std::vector<std::vector<cv::DMatch> > matches;
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 5);

    // 3. Remove matches for which NN ratio is > than threshold
    ratioTest2(matches);

    // 4. Fill good matches container
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        if (!matchIterator->empty()) {
            for (int i = 1; i <10 ; ++i) {
                good_matches.push_back((*matchIterator)[i]);

            }
        }
    }

    // if (!training_img_.empty() && !keypoints_model.empty())
    // {
    //     cv::drawMatches(frame, keypoints_frame, training_img_, keypoints_model, good_matches, img_matching_);
    // }
}


void RobustMatcher::fastRobustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                     std::vector<cv::KeyPoint>& keypoints_frame,
                                     const cv::Mat& descriptors_model)
{
    good_matches.clear();

    // 1a. Detection of the ORB features
    this->computeKeyPoints(frame, keypoints_frame);
    // std::cout<<"7777777777777777777777777777777777777"<<std::endl;
    // std::cout<<keypoints_frame[0].size<<std::endl;
    // std::cout<<keypoints_frame[0].angle<<std::endl;
    // std::cout<<keypoints_frame[0].response<<std::endl;
    // std::cout<<keypoints_frame[0].octave<<std::endl;
    // std::cout<<keypoints_frame[0].class_id<<std::endl;
    // std::cout<<"7777777777777777777777777777777777777"<<std::endl;
    // 1b. Extraction of the ORB descriptors
    cv::Mat descriptors_frame;
    this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

    // 2. Match the two image descriptors
    std::vector<std::vector<cv::DMatch> > matches;
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2);

    // 3. Remove matches for which NN ratio is > than threshold
    ratioTest(matches);

    // 4. Fill good matches container
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }

    // if (!training_img_.empty() && !keypoints_model.empty())
    // {
    //     cv::drawMatches(frame, keypoints_frame, training_img_, keypoints_model, good_matches, img_matching_);
    // }
}

void RobustMatcher::fastRobustMatchmap(std::vector<cv::DMatch>& good_matches,
                                     std::vector<cv::KeyPoint>& keypoints_frame,
                                     const cv::Mat& descriptors_frame,
                                     const cv::Mat& descriptors_model,
                                     const std::vector<cv::KeyPoint>& keypoints_model)
{
    good_matches.clear();

    // 1a. Detection of the ORB features
    // this->computeKeyPoints(frame, keypoints_frame);

    // // 1b. Extraction of the ORB descriptors
    // cv::Mat descriptors_frame;
    // this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

    // 2. Match the two image descriptors
    std::vector<std::vector<cv::DMatch> > matches;
    matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2);

    // 3. Remove matches for which NN ratio is > than threshold
    ratioTest(matches);

    // 4. Fill good matches container
    for ( std::vector<std::vector<cv::DMatch> >::iterator
          matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }

    if (!training_img_.empty() && !keypoints_model.empty())
    {
        std::cout<<"drawmatches!!!"<<std::endl;
        // cv::drawMatches(frame, keypoints_frame, training_img_, keypoints_model, good_matches, img_matching_);
    }
}
