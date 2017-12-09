//
//  Frame.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#include "Frame.hpp"
#include <opencv2/core/core.hpp>

namespace MiniSlam {

cv::Ptr<cv::ORB> Frame::mDetector = cv::ORB::create();

Frame::Frame() {
        
}

Frame::Frame(cv::Mat im) {
    mIm = im.clone();
    mDetector->detect(im, mvKeyPoints);
    mDetector->compute(im, mvKeyPoints, mDescriptors);
}

}
