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
#include "ImuPreIntegration.hpp"

using namespace std;
using namespace MiniSlam;

struct ImuData {
    double timeStamp;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double accel_x;
    double accel_y;
    double accel_z;
};

typedef struct ImuData ImuData;

double current_time = -1;

void LoadImages(const string& imageFilePath, const string& imageTimePath,
                vector<string>& vstrImages, vector<double>& vTimeStamps);
 
void LoadImus(ifstream &fImus, double timeStamp, vector<ImuData>& vImuDatas);

int main(int argc, const char * argv[]) {
    // insert code here...
    if (argc != 5) {
        cout << "MiniSlam path_to_settings image_folder time_stamp_file imu_data_file" << endl;
        return 1;
    }

    ifstream fImus;
    fImus.open(argv[4]);
    
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
    
    ImuPreIntegration* pre_integration = NULL;
    for (int ni=0; ni<nImages; ni++) {
        cout << "process i: " << ni << endl;
        double tframe = vImageTimeStamps[ni];
        vector<ImuData> vImuDatas;
        LoadImus(fImus, tframe, vImuDatas);
        for (int i=0; i<vImuDatas.size(); i++)
        {
            if (!pre_integration)
                pre_integration = new ImuPreIntegration(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            ImuData data = vImuDatas[i];
            if (current_time < 0)
                current_time = data.timeStamp;
            double dt = data.timeStamp - current_time;
            current_time = data.timeStamp;
            pre_integration->push_back(dt, Eigen::Vector3d(data.accel_x, data.accel_y, data.accel_z), Eigen::Vector3d(data.gyro_x, data.gyro_y, data.gyro_z));
            //cout << "time: " << dt << " accel (" << data.accel_x << "," << data.accel_y << "," << data.accel_z << ")" << endl;
            //cout << "gyro (" << data.gyro_x << "," << data.gyro_y << "," << data.gyro_z << ")" << endl;
        }
        //cout << "read imu : " << vImuDatas.size() << endl;
        //cout << "p: " << pre_integration->get_delta_p() << endl << " v: " << pre_integration->get_delta_v() << endl;
        delete pre_integration;
        pre_integration = NULL;
        im = cv::imread(vImageFileNames[ni], CV_LOAD_IMAGE_UNCHANGED);
        if (im.empty()) {
            cout << "image is empty: " << vImageFileNames[ni] << endl;
            return 1;
        }
        /* cv::imshow("image", im);
        int k = cv::waitKey(20);
        if (k>0)
            break; */
        mSystem.ProcessFrame(im, tframe);
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

void LoadImus(ifstream &fImus, double timeStamp, vector<ImuData> &vImuDatas)
{
    while(!fImus.eof())
    {
        string s;
        getline(fImus, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c<'0' || c>'9')
                continue;
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[7];
            while (ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }

            ImuData imu;
            imu.timeStamp = data[0] * 1e-9;
            imu.gyro_x = data[1];
            imu.gyro_y = data[2];
            imu.gyro_z = data[3];
            imu.accel_x = data[4];
            imu.accel_y = data[5];
            imu.accel_z = data[6];
            vImuDatas.push_back(imu);
            
            if (imu.timeStamp > timeStamp)
                break;
        }
    }
}
