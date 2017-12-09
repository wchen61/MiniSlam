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
    Frame(cv::Mat im);

    vector<cv::KeyPoint> mvKeyPoints;
    cv::Mat mDescriptors;
    cv::Mat mTcw;
    cv::Mat mIm;
private:
    static cv::Ptr<cv::ORB> mDetector;
};

}
#endif /* Frame_hpp */
