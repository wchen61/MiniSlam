//
//  Frame.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#include "Frame.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_CNT 150
#define MIN_DIST 30

namespace MiniSlam {
int Frame::n_id = 0;
cv::Ptr<cv::ORB> Frame::mDetector = cv::ORB::create();

Frame::Frame() {
}

Frame::Frame(const cv::Mat &im, const double &timestamp, cv::Mat &K, cv::Mat &distCoef) :
    mK(K.clone()), mDistCoef(distCoef.clone()) {
    mIm = im.clone();
    mPts.clear();
    mTimestamp = timestamp;
    //mDetector->detect(im, mvKeyPoints);
    //mDetector->compute(im, mvKeyPoints, mDescriptors);
}

void Frame::AddFeatures() {
    int n_max_cnt = MAX_CNT - static_cast<int>(mPts.size());
    if (n_max_cnt <= 0)
        return;
    
    cv::Mat mask;
    mask = cv::Mat(mIm.rows, mIm.cols, CV_8UC1, cv::Scalar(255));

    for (auto &it : mPts) {
        if (mask.at<uchar>(it) == 255)
        {
            cv::circle(mask, it, MIN_DIST, 0, -1);
        }
    }
    
    vector<cv::Point2f> n_pts;
    cv::goodFeaturesToTrack(mIm, n_pts, n_max_cnt, 0.1, MIN_DIST, mask);
    for (auto &p : n_pts) {
        mPts.push_back(p);
        mIds.push_back(n_id++);
        mTrack_cnt.push_back(1);
    }
}
    
void Frame::UndistortPoints() {
    cv::undistortPoints(mPts, mUnPts, mK, mDistCoef);
}

}
