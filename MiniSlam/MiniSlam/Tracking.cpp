//
//  Tracking.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/8.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#include "Tracking.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MAX_CNT 150
#define COL 752
#define ROW 480

namespace MiniSlam {
    
int Tracking::n_id = 0;
    
Tracking::Tracking()
{
    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = 4.616e+02;
    mK.at<float>(1,1) = 4.603e+02;
    mK.at<float>(0,2) = 3.630e+02;
    mK.at<float>(1,2) = 2.481e+02;
    
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    mDistCoef.at<float>(0) = -2.917e-01;
    mDistCoef.at<float>(1) = 8.229e-02;
    mDistCoef.at<float>(2) = 5.333e-05;
    mDistCoef.at<float>(3) = -1.578e-04;
    
    mEstimator = new Estimator();
}

void Tracking::GrabImage(const cv::Mat &im, const double &timestamp, bool publish)
{
    mImGray = im;
    if (mImGray.channels() == 3) {
        cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    }
    
    if (mCurrentFrame.mIm.empty()) {
        mCurrentFrame = Frame(mImGray, timestamp, mK, mDistCoef);
        mPrevFrame = Frame(mImGray, timestamp, mK, mDistCoef);
    } else {
        mCurrentFrame = Frame(mImGray, timestamp, mK, mDistCoef);
    }
    
    if (mPrevFrame.mPts.size() > 0) {
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(mPrevFrame.mIm, mCurrentFrame.mIm, mPrevFrame.mPts, mCurrentFrame.mPts, status, err, cv::Size(21,21), 3);
        for (auto id : mPrevFrame.mIds)
            mCurrentFrame.mIds.push_back(id);

        for (int i=0; i<int(mCurrentFrame.mPts.size()); i++)
            if (status[i] && !InBorder(mCurrentFrame.mPts[i]))
                status[i] = 0;
        ReduceVector(mCurrentFrame.mPts, status);
        ReduceVector(mCurrentFrame.mIds, status);
    }
    
    if (publish) {
        mCurrentFrame.AddFeatures();
        mEstimator->ProcessImage(mCurrentFrame);
    }

    mPrevFrame = mCurrentFrame;
    
    cv::Mat tmp_img = mImGray;
    cv::cvtColor(mImGray, tmp_img, CV_GRAY2RGB);
    for (unsigned int j=0; j<mCurrentFrame.mPts.size(); j++)
    {
        cv::circle(tmp_img, mCurrentFrame.mPts[j], 2, cv::Scalar(0, 0, 255), 2);
    }
    cv::namedWindow("MiniSlam", CV_WINDOW_NORMAL);
    cv::imshow("MiniSlam", tmp_img);
    int k = cv::waitKey(5);
    if (k>0)
        exit(1);
}
    
bool Tracking::InBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}
    
void Tracking::RejectWithF() {
    
}

/*void Tracking::ReduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j=0;
    for (int i=0; i<int(v.size()); i++)
        if (status[i])
            v[j++]=v[i];
    v.resize(j);
}*/

}
