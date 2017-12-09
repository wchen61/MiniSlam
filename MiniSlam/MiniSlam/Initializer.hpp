//
//  Initializer.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#ifndef Initializer_hpp
#define Initializer_hpp

#include <stdio.h>

#include "Frame.hpp"

namespace MiniSlam {
class Initializer {
public:
    Initializer() : mFirstFrame(false) {};
    bool Initialize(Frame f);

private:
    Frame mInitialFrame;
    Frame mRefFrame;
    vector<cv::DMatch> mMatches;
    vector<cv::DMatch> mGoodMatches;
    cv::BFMatcher mBFMatcher;
    bool mFirstFrame;
};

}

#endif /* Initializer_hpp */
