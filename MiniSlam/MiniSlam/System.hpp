//
//  System.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#ifndef System_hpp
#define System_hpp

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include "Frame.hpp"
#include "Initializer.hpp"
#include "Tracking.hpp"

using namespace cv;

namespace MiniSlam {
class System {
public:
    System();
    void ProcessFrame(const cv::Mat &im, const double &timestamp);
    void Initialize();
    
    enum SLAM_STATE {
        NOT_INITIALIZED = 0,
        INITIALIZING = 1,
        TRACKING = 2,
        LOST = 3,
    };

protected:
    enum SLAM_STATE mState;
    Initializer mInitializer;
    Tracking* mTracking;
    bool mPubFrame;
    double mFirstPubTime;
    int mFreq;
    bool mFirstImage;
    int mPubFrameCount;
};
}
#endif /* System_hpp */
