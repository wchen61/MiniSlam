//
//  ImuPreIntegration.hpp
//  MiniSlam
//
//  Created by  weiwu.cww on 2018/6/7.
//  Copyright © 2018年  weiwu.cww. All rights reserved.
//

#ifndef ImuPreIntegration_hpp
#define ImuPreIntegration_hpp

#include <stdio.h>
#include <vector>
#include <eigen3/Eigen/Dense>


using namespace Eigen;
namespace MiniSlam {
class ImuPreIntegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuPreIntegration() = delete;
    ImuPreIntegration(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                      const Eigen::Vector3d &_ba, const Eigen::Vector3d& _bg);
    
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    
    Eigen::Vector3d get_delta_p() {return delta_p;}
    Eigen::Quaterniond get_delta_q() {return delta_q;}
    Eigen::Vector3d get_delta_v() {return delta_v;}

private:
    void propagate(double _dt, const Eigen::Vector3d &_acc, const Eigen::Vector3d &_gyr);
    void integration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                     const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                     const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                     const Eigen::Vector3d &ba, const Eigen::Vector3d &bg,
                     Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                     Eigen::Vector3d &result_ba, Eigen::Vector3d &result_bg);
    
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;
    
    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    
    std::vector<double> vDtBuf;
    std::vector<Eigen::Vector3d> vAccBuf;
    std::vector<Eigen::Vector3d> vGyrBuf;
};
}
#endif /* ImuPreIntegration_hpp */
