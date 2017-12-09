//
//  main.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/6.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#include <iostream>
#include "fstream"
#include "unistd.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Time.h"
#include "System.hpp"

using namespace std;
using namespace MiniSlam;

void LoadImages(const string& imageFilePath, const string& imageTimePath,
                vector<string>& vstrImages, vector<double>& vTimeStamps);

int main(int argc, const char * argv[]) {
    // insert code here...
    if (argc != 4) {
        cout << "MiniSlam path_to_settings image_folder time_stamp_file" << endl;
        return 1;
    }
    
    System mSystem = System();

    vector<string> vImageFileNames;
    vector<double> vImageTimeStamps;

    LoadImages(argv[2], argv[3], vImageFileNames, vImageTimeStamps);
    size_t nImages = vImageFileNames.size();
    
    if (nImages <= 0) {
        cout << "No images files" << endl;
        return 1;
    }
    
    cout << "process images " << nImages << endl;

    namedWindow("image", cv::WINDOW_AUTOSIZE);
    vector<float> vTimeTrack;
    vTimeTrack.resize(nImages);
    cv::Mat im;
    for (int ni=0; ni<nImages; ni++) {
        cout << "process i: " << ni << endl;
        double tframe = vImageTimeStamps[ni];
        im = cv::imread(vImageFileNames[ni]);
        if (im.empty()) {
            cout << "image is empty: " << vImageFileNames[ni] << endl;
            return 1;
        }
        cv::imshow("image", im);
        int k = cv::waitKey(20);
        if (k>0)
            break;
        mSystem.ProcessFrame(im);
        double T = vImageTimeStamps[ni+1] - tframe;
        cout << "sleep time : " << T << endl;
        usleep(T*1e6);
    }
    
    /*cv::VideoCapture capture(0);
    cv::Mat im;
    while (1) {
        capture >> im;
        if (im.empty()) {
            cout << "image is empty" << endl;
            return 1;
        }
        
        cv::imshow("MiniSlam", im);
        int k = cv::waitKey(20);
        if (k>0)
            return 1;
        //cv::waitKey();
    }*/
    
    return 0;
}

void LoadImages(const string& imageFilePath, const string& imageTimePath,
                vector<string>& vstrImages, vector<double>& vTimeStamps) {
    ifstream ftimes;
    ftimes.open(imageTimePath.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    cout << "file path: " << imageFilePath << endl;
    cout << "time file: "<< ftimes.is_open() <<  " path: " << imageTimePath << endl;
    while(!ftimes.eof()) {
        string s;
        getline(ftimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(imageFilePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
    cout << "size: " << vstrImages.size() << endl;
}
