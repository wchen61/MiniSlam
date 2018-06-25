//
//  Tracking.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/8.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#ifndef Tracking_hpp
#define Tracking_hpp

#include <stdio.h>
#include "Frame.hpp"
#include "Estimator.hpp"

namespace MiniSlam {
class Tracking {
public:
    Tracking();

    void GrabImage(const cv::Mat &im, const double &timestamp, bool publish);
    void RejectWithF();

private:
    bool InBorder(const cv::Point2f &pt);
    //void ReduceVector(vector<cv::Point2f> &v, vector<uchar> status);
    
    template <typename T>
    static void ReduceVector(vector<T> &v, vector<uchar> status)
    {
        int j=0;
        for (int i=0; i<int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    Frame mPrevFrame;
    Frame mCurrentFrame;

    cv::Mat mK;
    cv::Mat mDistCoef;

    static int n_id;
    cv::Mat mImGray;
    Estimator* mEstimator;
};
}
#endif /* Tracking_hpp */
