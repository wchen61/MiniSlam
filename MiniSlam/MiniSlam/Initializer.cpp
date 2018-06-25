//
//  Initializer.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/12.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#include "Initializer.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "stdio.h"
#include <iostream>

using namespace std;

namespace MiniSlam {

bool Initializer::Initialize(Frame f) {
    if (!mFirstFrame) {
        mInitialFrame = Frame(f);
        mFirstFrame = true;
        return false;
    } else {
        /*mRefFrame = Frame(f);
        mMatches.clear();
        mGoodMatches.clear();
        mBFMatcher.match(mInitialFrame.mDescriptors, mRefFrame.mDescriptors, mMatches);
        std::cout << "find matches: " << mMatches.size() << endl;
        cv::Mat imageMatches;
        cv::drawMatches(mInitialFrame.mIm, mInitialFrame.mvKeyPoints, mRefFrame.mIm, mRefFrame.mvKeyPoints, mMatches, imageMatches);
        cv::imshow("matches", imageMatches);
        //cv::waitKey();
        
        double min_dist = 1000;
        double max_dist = 0;
        for (int i=0; i<mInitialFrame.mDescriptors.rows; i++) {
            double dist = mMatches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        
        std::cout << "max distance: " << max_dist << " min distance: " << min_dist << endl;
        
        for (int i=0; i<mInitialFrame.mDescriptors.rows; i++) {
            if (mMatches[i].distance <= max(2*min_dist, 30.0)) {
                mGoodMatches.push_back(mMatches[i]);
            }
        }
        
        cout << "good matches " << mGoodMatches.size() << endl;
        cv::drawMatches(mInitialFrame.mIm, mInitialFrame.mvKeyPoints, mRefFrame.mIm, mRefFrame.mvKeyPoints, mGoodMatches, imageMatches);
        cv::imshow("matches", imageMatches);
        //cv::waitKey();
        
        vector<cv::Point2f> points1;
        vector<cv::Point2f> points2;
        
        for (int i=0; i<mGoodMatches.size(); i++) {
            int queryIdx = mGoodMatches[i].queryIdx;
            int trainIdx = mGoodMatches[i].trainIdx;
            points1.push_back(mInitialFrame.mvKeyPoints[queryIdx].pt);
            points2.push_back(mRefFrame.mvKeyPoints[trainIdx].pt);
        }
        
        cv::Mat F;
        cv::Mat H;
        F = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
        cout << F << endl;
        H = cv::findHomography(points1, points2, cv::RANSAC, 3);
        cout << H << endl;*/

        return true;
    }
    return true;
}

}
