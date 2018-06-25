//
//  System.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#include "System.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

namespace MiniSlam {
System::System() : mPubFrame(false), mFirstPubTime(0.0f),
    mFreq(10), mFirstImage(true), mPubFrameCount(0) {
    mState = NOT_INITIALIZED;
    mTracking = new Tracking();
}

void System::ProcessFrame(const cv::Mat &im, const double &timestamp)
{
    if (mState == NOT_INITIALIZED) {
        Initialize();
        mState = INITIALIZING;
    }
    
    if (mFirstImage) {
        mFirstImage = false;
        mFirstPubTime = timestamp;
    }
    
    if (round(1.0 * mPubFrameCount / ((timestamp - mFirstPubTime) * 10e-6)) <= mFreq) {
        mPubFrame = true;
        if (abs(1.0 * mPubFrameCount / ((timestamp - mFirstPubTime) * 10e-6) - mFreq) < 0.01 * mFreq) {
            mFirstPubTime = timestamp;
            mPubFrameCount = 0;
        }
    } else {
        mPubFrame = false;
    }
    
    mTracking->GrabImage(im, timestamp, mPubFrame);
    //mInitializer.Initialize(mCurrentFrame);
}

void System::Initialize()
{

}

}
