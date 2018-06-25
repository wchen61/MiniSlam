//
//  ImuPreIntegration.cpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/7.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#include "ImuPreIntegration.hpp"

namespace MiniSlam {
ImuPreIntegration::ImuPreIntegration(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                  const Eigen::Vector3d &_ba, const Eigen::Vector3d& _bg)
    : acc_0(_acc_0), gyr_0(_gyr_0), linearized_acc(_acc_0), linearized_gyr(_gyr_0),
      linearized_ba(_ba), linearized_bg(_bg), sum_dt(0.0), delta_p{(Eigen::Vector3d::Zero())},
      delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}
{
}

void ImuPreIntegration::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    vDtBuf.push_back(dt);
    vAccBuf.push_back(acc);
    vGyrBuf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void ImuPreIntegration::propagate(double _dt, const Eigen::Vector3d &_acc, const Eigen::Vector3d &_gyr)
{
    dt = _dt;
    acc_1 = _acc;
    gyr_1 = _gyr;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_ba;
    Vector3d result_bg;
    integration(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
                linearized_ba, linearized_bg, result_delta_p, result_delta_q,
                result_delta_v, result_ba, result_bg);
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_ba;
    linearized_bg = result_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
}

void ImuPreIntegration::integration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                                    const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                                    const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                    const Eigen::Vector3d &ba, const Eigen::Vector3d &bg,
                                    Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                    Eigen::Vector3d &result_ba, Eigen::Vector3d &result_bg)
{
    /*result_delta_p = delta_p + delta_v * _dt + 0.5 *(delta_q *(_acc_1 - ba))*_dt*_dt;
    result_delta_v = delta_v + delta_q*(_acc_1-ba)*_dt;
    Eigen::Vector3d omg = 0.5 * (_gyr_1 - bg) * _dt;
    Quaterniond dR(1, omg(0), omg(1), omg(2));
    result_delta_q = delta_q * dR;
    result_ba = ba;
    result_bg = bg;*/
    
    Vector3d g = Vector3d(9.815, 0, 0);
    Vector3d un_acc_0 = delta_q * (_acc_0 - ba - delta_q.inverse() * g);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - bg;
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0)*_dt/2, un_gyr(1)*_dt/2, un_gyr(2)*_dt/2);
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - ba - result_delta_q.inverse() * g);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc *_dt;
    result_ba = ba;
    result_bg = bg;
}
}
