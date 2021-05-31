#ifndef EXTRINSICFACTOR
#define EXTRINSICFACTOR
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include "ncrl_lio/ncrl_lio_parameters_fusion.h"

class ExtrinFactor : public ceres::SizedCostFunction<6, 7>
{
  public:
    ExtrinFactor() = delete;
    ExtrinFactor(Eigen::Vector3d Ex_trans_last_,
                 Eigen::Quaterniond Ex_quat_last_,
                 double T_weight_,
                 double R_weight_):
                 Ex_trans_last(Ex_trans_last_),
                 Ex_quat_last(Ex_quat_last_),
                 T_weight(T_weight_),
                 R_weight(R_weight_){}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Ex_trans{parameters[0][0], parameters[0][1], parameters[0][2]};
        Eigen::Quaterniond Ex_Quat{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        Eigen::Matrix<double, 6, 6> sqrt_info_;
        sqrt_info_.setZero(6, 6);
        sqrt_info_.topLeftCorner<3, 3>() = Matrix3d::Identity() * T_weight;
        sqrt_info_.bottomRightCorner<3, 3>() = Matrix3d::Identity() * R_weight;

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.topRows<3>() = Ex_trans - Ex_trans_last;
        residual.bottomRows<3>() = 2 * (Ex_quat_last.inverse() * Ex_Quat).vec();

        residual = sqrt_info_ * residual;

        if (jacobians) {
          if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_extrin(jacobians[0]);
            Eigen::Matrix<double, 6, 6> jaco_extrin;
            jaco_extrin.setIdentity();
            jaco_extrin.bottomRightCorner<3, 3>() = Utility::Qleft(Ex_quat_last.inverse() * Ex_Quat).bottomRightCorner<3, 3>();

            jacobian_extrin.setZero();
            jacobian_extrin.leftCols<6>() = sqrt_info_ * jaco_extrin;
          }
        }

      return true;
    }

    Eigen::Vector3d Ex_trans_last;
    Eigen::Quaterniond Ex_quat_last;
    double T_weight, R_weight;
};

#endif // EXTRINSICFACTOR
