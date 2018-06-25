//
//  Estimator.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/8.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#ifndef Estimator_hpp
#define Estimator_hpp

#include <stdio.h>
#include <eigen3/Eigen/Geometry>
#include <map>
#include "Frame.hpp"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace MiniSlam
{

struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool state;
    vector<pair<int, Eigen::Vector2d>> observation;
    double position[3];
    double depth;
};

struct ReprojectionError3D
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReprojectionError3D(double u, double v) :
        observed_u(u), observed_v(v) {}
   
    template<typename T>
    bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
    {
        T p[3];
        ceres::QuaternionRotatePoint(camera_R, point, p);
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        T xp = p[0]/p[2];
        T yp = p[0]/p[2];
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        return true;
    }

    static ceres::CostFunction* Create(const double x, const double y)
    {
        return (new ceres::AutoDiffCostFunction<
                ReprojectionError3D, 2, 4, 3, 3>(
                    new ReprojectionError3D(x, y)));
    }

    double observed_u;
    double observed_v;
};
    
class Estimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Estimator();
    void ProcessImage(const Frame &frame);
    bool InitialStructure();
    bool RelativePose(Eigen::Matrix3d &relativeR, Eigen::Vector3d &relativeT, int &l);

private:
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> GetCorresponding(int frame_l, int frame_r);
    bool SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres, Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation);
    
    bool Construct(int frame_num, Eigen::Quaterniond *q, Eigen::Vector3d *T, int l,
                   const Eigen::Matrix3d relative_R, const Eigen::Vector3d relative_T);
    void TriangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1);
    
    void TriangulatePoint(Eigen::Matrix<double, 3, 4> &pose0, Eigen::Matrix<double, 3, 4> &pose1,
                          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    
    bool SolveFrameByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &T, int i);

    enum EstimatorState {
        INITIAL,
        TRAKING
    };

    EstimatorState mState;
    
    vector<Frame> mSlideWindow;
    std::map<int, MapPoint> mMap;
    int mFrameCount;
};
    
}

#endif /* Estimator_hpp */
