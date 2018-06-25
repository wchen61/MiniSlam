//
//  Frame.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#ifndef Frame_hpp
#define Frame_hpp

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
namespace MiniSlam {

class Frame {
public:
    Frame();
    Frame(const cv::Mat &im, const double &timestamp, cv::Mat &K, cv::Mat &distCoef);
    
    void AddFeatures();
    void UndistortPoints();

    /*vector<cv::KeyPoint> mvKeyPoints;
    cv::Mat mDescriptors;
    cv::Mat mTcw;
    cv::Mat mIm;*/
 
    cv::Mat mIm;
    vector<cv::Point2f> mPts;
    vector<cv::Point2f> mUnPts;
    vector<int> mIds;
    vector<int> mTrack_cnt;
    double mTimestamp;

private:
    static int n_id;
    static cv::Ptr<cv::ORB> mDetector;
    cv::Mat mK;
    cv::Mat mDistCoef;
};

}
#endif /* Frame_hpp */
