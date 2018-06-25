//
//  Estimator.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/8.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#include "Estimator.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

const int WINDOW_SIZE = 10;

using namespace Eigen;

namespace MiniSlam {

Estimator::Estimator()
{
    mState = INITIAL;
}

void Estimator::ProcessImage(const MiniSlam::Frame &frame)
{
    for (int i=0; i<frame.mPts.size(); i++)
    {
        if (mMap.count(frame.mIds[i])) {
            mMap[frame.mIds[i]].observation.push_back(std::make_pair(mFrameCount, Eigen::Vector2d(frame.mPts[i].x, frame.mPts[i].y)));
        } else {
            MapPoint mp;
            mp.state = false;
            mp.observation.push_back(std::make_pair(mFrameCount, Eigen::Vector2d(frame.mPts[i].x, frame.mPts[i].y)));
            mMap[frame.mIds[i]] = mp;
        }
    }

    mSlideWindow.push_back(frame);
    if (mState == INITIAL) {
        if (mFrameCount == WINDOW_SIZE) {
            bool result = false;
            result = InitialStructure();
        } else {
            mFrameCount++;
        }
    } else {
        
    }
}
    
bool Estimator::InitialStructure()
{
    Eigen::Quaterniond Q[mFrameCount+1];
    Eigen::Vector3d T[mFrameCount+1];
    
    Eigen::Matrix3d relative_R;
    Eigen::Vector3d relative_T;
    int l;
    if (!RelativePose(relative_R, relative_T, l))
        return false;
    
    std::cout << "relative_R:" << std::endl;
    std::cout << relative_R << std::endl;
    std::cout << "relative_T:" << std::endl;
    std::cout << relative_T << std::endl;
    std::cout << "l: " << l << std::endl;
    
    if (!Construct(mFrameCount+1, Q, T, l, relative_R, relative_T))
        return false;
    
    return true;
}

bool Estimator::RelativePose(Eigen::Matrix3d &relativeR, Eigen::Vector3d &relativeT, int &l)
{
    for (int i=0; i<mFrameCount; i++) {
        vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = GetCorresponding(i, mFrameCount);
        if (corres.size() > 20) {
            double sum_parallax = 0;
            double average_parallax;
            for (int j=0; j<int(corres.size()); j++) {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax += parallax;
            }
            average_parallax = 1.0 * sum_parallax / (int)(corres.size());
            if (average_parallax * 460 > 30 && SolveRelativeRT(corres, relativeR, relativeT)) {
                l = i;
                return true;
            }
        }
    }
    return false;
}

vector<pair<Eigen::Vector3d, Eigen::Vector3d>> Estimator::GetCorresponding(int frame_l, int frame_r)
{
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    for (int i=0; i<mSlideWindow[frame_l].mIds.size(); i++) {
        for (int j=0; j<mSlideWindow[frame_r].mIds.size(); j++) {
            if (mSlideWindow[frame_l].mIds[i] == mSlideWindow[frame_r].mIds[j]) {
                Eigen::Vector3d a = Eigen::Vector3d(mSlideWindow[frame_l].mPts[i].x, mSlideWindow[frame_l].mPts[i].y, 1);
                Eigen::Vector3d b = Eigen::Vector3d(mSlideWindow[frame_r].mPts[j].x, mSlideWindow[frame_r].mPts[j].y, 1);
                corres.push_back(make_pair(a, b));
            }
        }
    }
    return corres;
}
    
bool Estimator::SolveRelativeRT(const vector<pair<Eigen::Vector3d, Eigen::Vector3d> > corres, Eigen::Matrix3d &Rotation, Eigen::Vector3d &Translation)
{
    if (corres.size() >= 15) {
        vector<cv::Point2f> ll, rr;
        for (int i=0; i<int(corres.size()); i++) {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat mask;
        cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3/460, 0.99, mask);
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3)<< 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rot, trans;
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
        
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        for (int i=0; i<3; i++) {
            T(i) = trans.at<double>(i, 0);
            for (int j=0; j<3; j++)
                R(i, j) = rot.at<double>(i, j);
        }
        
        Rotation = R.transpose();
        Translation = -R.transpose() * T;
        if (inlier_cnt > 12)
            return true;
        else
            return false;
    }
    return false;
}
    
bool Estimator::Construct(int frame_num, Eigen::Quaterniond *q, Eigen::Vector3d *T, int l, const Eigen::Matrix3d relative_R, const Eigen::Vector3d relative_T)
{
    q[l].w() = 1;
    q[l].x() = 0;
    q[l].y() = 0;
    q[l].z() = 0;
    T[l].setZero();
    q[frame_num - 1] = q[l] * Quaterniond(relative_R);
    T[frame_num - 1] = relative_T;
    
    Matrix3d c_Rotation[frame_num];
    Vector3d c_Translation[frame_num];
    Quaterniond c_Quat[frame_num];
    double c_rotation[frame_num][4];
    double c_translation[frame_num][3];
    Eigen::Matrix<double, 3, 4> Pose[frame_num];
    c_Quat[l] = q[l].inverse();
    c_Rotation[l] = c_Quat[l].toRotationMatrix();
    c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];
    
    c_Quat[frame_num - 1] = q[frame_num -1].inverse();
    c_Rotation[frame_num -1] = c_Quat[frame_num - 1].toRotationMatrix();
    c_Translation[frame_num - 1] = -1 *(c_Rotation[frame_num - 1] * T[frame_num - 1]);
    Pose[frame_num - 1].block<3,3>(0,0) = c_Rotation[frame_num - 1];
    Pose[frame_num - 1].block<3,1>(0,3) = c_Translation[frame_num - 1];
    
    for (int i=l; i<frame_num -1; i++) {
        if (i>l) {
            Matrix3d R_initial = c_Rotation[i-1];
            Vector3d P_initial = c_Translation[i-1];
            if (!SolveFrameByPnP(R_initial, P_initial, i))
                return false;
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3,3>(0,0) = c_Rotation[i];
            Pose[i].block<3,1>(0,3) = c_Translation[i];
        }
        TriangulateTwoFrames(i, Pose[i], frame_num-1, Pose[frame_num - 1]);
    }
    
    for (int i= l+1; i<frame_num-1; i++)
        TriangulateTwoFrames(l, Pose[l], i, Pose[i]);

    for (int i=l-1; i>=0; i--) {
        Matrix3d R_initial = c_Rotation[i+1];
        Vector3d P_initial = c_Translation[i+1];
        if (!SolveFrameByPnP(R_initial, P_initial, i))
            return false;
        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3,3>(0,0) = c_Rotation[i];
        Pose[i].block<3,1>(0,3) = c_Translation[i];
        TriangulateTwoFrames(i, Pose[i], l, Pose[l]);
    }
    
    for (map<int, MapPoint>::iterator it = mMap.begin(); it != mMap.end(); it++) {
        if (it->second.state == true)
            continue;
        if (it->second.observation.size() >= 2) {
            Vector2d point0, point1;
            int frame_0 = it->second.observation[0].first;
            point0 = it->second.observation[0].second;
            int frame_1 = it->second.observation.back().first;
            point1 = it->second.observation.back().second;
            Vector3d point_3d;
            TriangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
            it->second.state = true;
            it->second.position[0] = point_3d(0);
            it->second.position[1] = point_3d(1);
            it->second.position[2] = point_3d(2);
        }
    }
    
    for (int i=0; i<frame_num; i++)
    {
        q[i] = c_Rotation[i].transpose();
        cout << "SolvePnP q " << " i " << i << " " << q[i].w() << " " << q[i].vec().transpose() << endl;
    }
    for (int i=0; i<frame_num; i++)
    {
        Vector3d t_tmp;
        t_tmp = -1 *(q[i] * c_Translation[i]);
        cout << "SolvePnP t" << "i " << i << " " << t_tmp.x() << " " << t_tmp.y() << " "  << t_tmp.z() << endl;
    }
    
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    for (int i=0; i<frame_num; i++)
    {
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);
        if (i==l) {
            problem.SetParameterBlockConstant(c_rotation[i]);
        }
        if (i==l || i == frame_num - 1) {
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }
    
    for (map<int, MapPoint>::iterator it = mMap.begin(); it != mMap.end(); it++) {
        if (it->second.state != true)
            continue;
        for (int j=0; j<it->second.observation.size(); j++) {
            int frame_index = it->second.observation[j].first;
            ceres::CostFunction *cost_function = ReprojectionError3D::Create(it->second.observation[j].second.x(),
                                                                             it->second.observation[j].second.y());
            problem.AddResidualBlock(cost_function, NULL, c_rotation[frame_index], c_translation[frame_index], it->second.position);
        }
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
        cout << "vision only BA convergence" << endl;
    } else {
        cout << "vision only BA not convergence" << endl;
        return false;
    }
    
    for (int i=0; i<frame_num; i++) {
        q[i].w() = c_rotation[i][0];
        q[i].x() = c_rotation[i][1];
        q[i].y() = c_rotation[i][2];
        q[i].z() = c_rotation[i][3];
        q[i] = q[i].inverse();
    }
    
    for (int i=0; i<frame_num; i++) {
        T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
    }
    
    exit(0);
    
    return true;
}
    
void Estimator::TriangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, int frame1, Eigen::Matrix<double, 3, 4> &Pose1)
{
    for (map<int, MapPoint>::iterator it = mMap.begin(); it != mMap.end(); it++) {
        if (it->second.state)
            continue;
        vector<pair<int, Eigen::Vector2d>> observation = it->second.observation;
        
        bool has_0 = false, has_1 = false;
        Vector2d point0;
        Vector2d point1;
        for (int i=0; i<observation.size(); i++) {
            if (observation[i].first == frame0) {
                point0 = observation[i].second;
                has_0 = true;
            }
            if (observation[i].first == frame1) {
                point1 = observation[i].second;
                has_1 = true;
            }
        }
        if (has_0 && has_1)
        {
            Vector3d point_3d;
            TriangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            it->second.state = true;
            it->second.position[0] = point_3d(0);
            it->second.position[1] = point_3d(1);
            it->second.position[2] = point_3d(2);
        }
    }
}
    
void Estimator::TriangulatePoint(Eigen::Matrix<double, 3, 4> &pose0, Eigen::Matrix<double, 3, 4> &pose1, Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Matrix4d A = Matrix4d::Zero();
    A.row(0) = point0[0] * pose0.row(2) - pose0.row(0);
    A.row(1) = point0[1] * pose0.row(2) - pose0.row(1);
    A.row(2) = point1[0] * pose1.row(2) - pose1.row(0);
    A.row(3) = point1[1] * pose1.row(2) - pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool Estimator::SolveFrameByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &T, int i)
{
    vector<cv::Point2f> pts_2_vector;
    vector<cv::Point3f> pts_3_vector;
    for (map<int, MapPoint>::iterator it = mMap.begin(); it != mMap.end(); it++) {
        if (it->second.state != true)
            continue;
        for (int j=0; j<it->second.observation.size(); j++) {
            if (it->second.observation[j].first == i) {
                Vector2d img_pts = it->second.observation[j].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
                cv::Point3f pts_3(it->second.position[0], it->second.position[1], it->second.position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    
    if (int(pts_2_vector.size()) < 15) {
        return false;
    }
    
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(T, t);
    cv::Mat K = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ)
        return false;
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    R = R_pnp;
    T = T_pnp;
    return true;
}

}
