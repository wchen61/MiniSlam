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
System::System() {
    mState = NOT_INITIALIZED;
}

void System::ProcessFrame(cv::Mat im)
{
    mImGray = im;
    if (mImGray.channels() == 3) {
        cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    }

    if (mState == NOT_INITIALIZED) {
        Initialize();
        mState = INITIALIZING;
    }
    
    mCurrentFrame = Frame(im);
    mInitializer.Initialize(mCurrentFrame);
}

void System::Initialize()
{
    
}
}
