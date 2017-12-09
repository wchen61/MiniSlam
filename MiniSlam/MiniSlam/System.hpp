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

using namespace cv;

namespace MiniSlam {
class System {
public:
    System();
    void ProcessFrame(Mat im);
    void Initialize();
    
    enum SLAM_STATE {
        NOT_INITIALIZED = 0,
        INITIALIZING = 1,
        TRACKING = 2,
        LOST = 3,
    };

protected:
    enum SLAM_STATE mState;
    Mat mImGray;
    Frame mCurrentFrame;
    Initializer mInitializer;
};
}
#endif /* System_hpp */
