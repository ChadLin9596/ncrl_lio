#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include "utility/utility.h"
#include "ncrl_lio/ncrl_lio_parameters_fusion.h"

class LidarEdgeIJFactor : public ceres::SizedCostFunction<3, 7, 7, 7>
{
public:
  LidarEdgeIJFactor() = delete;
  LidarEdgeIJFactor(Eigen::Vector3d curr_point_,
                    Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_,
                    double s_,
                    double sqrt_info_):
                    curr_point(curr_point_),
                    last_point_a(last_point_a_),
                    last_point_b(last_point_b_),
                    s(s_),
                    sqrt_info(sqrt_info_) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d cp(curr_point.x(), curr_point.y(), curr_point.z());
        Eigen::Vector3d lpa(last_point_a.x(), last_point_a.y(), last_point_a.z());
        Eigen::Vector3d lpb(last_point_b.x(), last_point_b.y(), last_point_b.z());

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qil(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_j = qil * cp + til;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = Qj * point_j + Pj;
        //transform point from world to last body frame
        Eigen::Vector3d point_i = Qi.inverse() * (point_w - Pi);
        //transform point from last body to last lidar frame
        Eigen::Vector3d lp = qil.inverse() * (point_i - til);

        Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
        Eigen::Vector3d de = lpa - lpb;

        residuals[0] = sqrt_info * nu.x() / de.norm();
        residuals[1] = sqrt_info * nu.y() / de.norm();
        residuals[2] = sqrt_info * nu.z() / de.norm();

        if (jacobians) {
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose();
                Eigen::Matrix3d rot = - Utility::skewSymmetric(de) / de.norm() * ril.transpose() *
                                      Utility::skewSymmetric(Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi));

                jaco_x.leftCols<3>() = tran.block<1,3>(0,0);
                jaco_y.leftCols<3>() = tran.block<1,3>(1,0);
                jaco_z.leftCols<3>() = tran.block<1,3>(2,0);
                jaco_x.rightCols<3>() = rot.block<1,3>(0,0);
                jaco_y.rightCols<3>() = rot.block<1,3>(1,0);
                jaco_z.rightCols<3>() = rot.block<1,3>(2,0);

                jacobian_pose_i.setZero();
                jacobian_pose_i.block<1,6>(0,0) = sqrt_info * jaco_x;
                jacobian_pose_i.block<1,6>(1,0) = sqrt_info * jaco_y;
                jacobian_pose_i.block<1,6>(2,0) = sqrt_info * jaco_z;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose_j(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = - Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose();
                Eigen::Matrix3d rot = Utility::skewSymmetric(de) / de.norm() * ril.transpose() *
                                      Ri.transpose() * Rj * Utility::skewSymmetric(ril * cp + til);

                jaco_x.leftCols<3>() = tran.block<1,3>(0,0);
                jaco_y.leftCols<3>() = tran.block<1,3>(1,0);
                jaco_z.leftCols<3>() = tran.block<1,3>(2,0);
                jaco_x.rightCols<3>() = rot.block<1,3>(0,0);
                jaco_y.rightCols<3>() = rot.block<1,3>(1,0);
                jaco_z.rightCols<3>() = rot.block<1,3>(2,0);

                jacobian_pose_j.setZero();
                jacobian_pose_j.block<1,6>(0,0) = sqrt_info * jaco_x;
                jacobian_pose_j.block<1,6>(1,0) = sqrt_info * jaco_y;
                jacobian_pose_j.block<1,6>(2,0) = sqrt_info * jaco_z;
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_ex(jacobians[2]);
                Eigen::Matrix3d I3x3;
                I3x3.setIdentity();
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = - Utility::skewSymmetric(de) / de.norm() * ril.transpose()
                                       * (Ri.transpose() * Rj - I3x3);
                Eigen::Matrix3d rot = Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose() *
                                      Rj * ril * Utility::skewSymmetric(cp) -
                                      Utility::skewSymmetric(de) / de.norm() * Utility::skewSymmetric(ril.transpose()
                                      * (Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi) - til));

                jaco_x.leftCols<3>() = tran.block<1,3>(0,0);
                jaco_y.leftCols<3>() = tran.block<1,3>(1,0);
                jaco_z.leftCols<3>() = tran.block<1,3>(2,0);
                jaco_x.rightCols<3>() = rot.block<1,3>(0,0);
                jaco_y.rightCols<3>() = rot.block<1,3>(1,0);
                jaco_z.rightCols<3>() = rot.block<1,3>(2,0);

                jacobian_ex.setZero();
                jacobian_ex.block<1,6>(0,0) = sqrt_info * jaco_x;
                jacobian_ex.block<1,6>(1,0) = sqrt_info * jaco_y;
                jacobian_ex.block<1,6>(2,0) = sqrt_info * jaco_z;
            }
        }

    return true;
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
    double sqrt_info;
};

class LidarEdgeFactor : public ceres::SizedCostFunction<3, 7, 7>
{
  public:
    LidarEdgeFactor() = delete;
    LidarEdgeFactor(Eigen::Vector3d curr_point_,
                    Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_,
                    double s_,
                    double sqrt_info_):
                    curr_point(curr_point_),
                    last_point_a(last_point_a_),
                    last_point_b(last_point_b_),
                    s(s_),
                    sqrt_info(sqrt_info_) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond q_w_curr(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Quaterniond q_identity(1, 0, 0, 0);
        q_w_curr = q_identity.slerp(s, q_w_curr);
        Eigen::Vector3d t_w_curr(s * parameters[0][0], s * parameters[0][1], s * parameters[0][2]);

        Eigen::Vector3d cp(curr_point.x(), curr_point.y(), curr_point.z());
        Eigen::Vector3d lpa(last_point_a.x(), last_point_a.y(), last_point_a.z());
        Eigen::Vector3d lpb(last_point_b.x(), last_point_b.y(), last_point_b.z());

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qil(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_b = qil * cp + til;
        //transform point from current body to world frame
        Eigen::Vector3d lp = q_w_curr * point_b + t_w_curr;

        Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
        Eigen::Vector3d de = lpa - lpb;

        residuals[0] = sqrt_info * nu.x() / de.norm();
        residuals[1] = sqrt_info * nu.y() / de.norm();
        residuals[2] = sqrt_info * nu.z() / de.norm();

        if (jacobians) {
          Eigen::Matrix3d Rj = q_w_curr.toRotationMatrix();
          if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);
            Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

            Eigen::Matrix3d tran = - Utility::skewSymmetric(de) / de.norm();
            Eigen::Matrix3d rot =
             Utility::skewSymmetric(de) * Rj * Utility::skewSymmetric(ril * cp + til) / de.norm();

            jaco_x.leftCols<3>() = tran.block<1,3>(0,0);
            jaco_y.leftCols<3>() = tran.block<1,3>(1,0);
            jaco_z.leftCols<3>() = tran.block<1,3>(2,0);
            jaco_x.rightCols<3>() = rot.block<1,3>(0,0);
            jaco_y.rightCols<3>() = rot.block<1,3>(1,0);
            jaco_z.rightCols<3>() = rot.block<1,3>(2,0);

            jacobian_pose_j.setZero();
            jacobian_pose_j.block<1,6>(0,0) = sqrt_info * jaco_x;
            jacobian_pose_j.block<1,6>(1,0) = sqrt_info * jaco_y;
            jacobian_pose_j.block<1,6>(2,0) = sqrt_info * jaco_z;
          }
          if (jacobians[1]) {
              Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_ex(jacobians[1]);
              Eigen::Matrix<double, 1, 6> jaco_exx, jaco_exy, jaco_exz;

              Eigen::Matrix3d ex_tran = - Utility::skewSymmetric(de) * Rj / de.norm();
//              Eigen::Matrix3d ex_rot =
//               Utility::skewSymmetric(de) * Rj * ril * Utility::skewSymmetric(ril * cp + til) / de.norm();
              Eigen::Matrix3d ex_rot =
               Utility::skewSymmetric(de) * Rj * ril * Utility::skewSymmetric(cp) / de.norm();
              jaco_exx.leftCols<3>() = ex_tran.block<1,3>(0,0);
              jaco_exy.leftCols<3>() = ex_tran.block<1,3>(1,0);
              jaco_exz.leftCols<3>() = ex_tran.block<1,3>(2,0);
              jaco_exx.rightCols<3>() = ex_rot.block<1,3>(0,0);
              jaco_exy.rightCols<3>() = ex_rot.block<1,3>(1,0);
              jaco_exz.rightCols<3>() = ex_rot.block<1,3>(2,0);

              jacobian_ex.setZero();
              jacobian_ex.block<1,6>(0,0) = sqrt_info * jaco_exx;
              jacobian_ex.block<1,6>(1,0) = sqrt_info * jaco_exy;
              jacobian_ex.block<1,6>(2,0) = sqrt_info * jaco_exz;
          }
        }

      return true;
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
    double sqrt_info;
};

class LidarPlaneFactor : public ceres::SizedCostFunction<1, 7, 7, 7>
{
public:
  LidarPlaneFactor() = delete;
  LidarPlaneFactor(Eigen::Vector3d curr_point_,
                   Eigen::Vector3d last_point_j_,
                   Eigen::Vector3d last_point_l_,
                   Eigen::Vector3d last_point_m_,
                   double s_,
                   double sqrt_info_):
                   curr_point(curr_point_),
                   last_point_j(last_point_j_),
                   last_point_l(last_point_l_),
                   last_point_m(last_point_m_),
                   s(s_),
                   sqrt_info(sqrt_info_)
  {
      ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
      ljm_norm.normalize();
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d cp(curr_point.x(), curr_point.y(), curr_point.z());
        Eigen::Vector3d lpj(last_point_j.x(), last_point_j.y(), last_point_j.z());
        Eigen::Vector3d ljm(ljm_norm.x(), ljm_norm.y(), ljm_norm.z());

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qil(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_j = qil * cp + til;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = Qj * point_j + Pj;
        //transform point from world to last body frame
        Eigen::Vector3d point_i = Qi.inverse() * (point_w - Pi);
        //transform point from last body to last lidar frame
        Eigen::Vector3d point_l = qil.inverse() * (point_i - til);
        residuals[0] = sqrt_info * (point_l - lpj).dot(ljm);

//        Eigen::Quaterniond q_identity{1, 0, 0, 0};
//        Eigen::Quaterniond q_last_curr = Qi.inverse() * Qj;
//        q_last_curr = q_identity.slerp(s, q_last_curr);
//        Eigen::Vector3d t_last_curr = Qi.inverse() * (Pj - Pi) * s;
//        Eigen::Vector3d lp ;
//        lp = q_last_curr * cp + t_last_curr;
//        residuals[0] = sqrt_info * (lp - lpj).dot(ljm);

        if (jacobians) {
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_i;

                jaco_i.leftCols<3>() = - ljm.transpose() * ril.transpose() * Ri.transpose();
                jaco_i.rightCols<3>() = ljm.transpose() * ril.transpose() *
                    Utility::skewSymmetric(Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi));

                jacobian_pose_i.setZero();
                jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
                jacobian_pose_i.rightCols<1>().setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_j(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_j;

                jaco_j.leftCols<3>() = ljm.transpose() * ril.transpose() * Ri.transpose();
                jaco_j.rightCols<3>() =
                 - ljm.transpose() * ril.transpose() * Ri.transpose() * Rj * Utility::skewSymmetric(ril * cp + til);

                jacobian_pose_j.setZero();
                jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_ex(jacobians[2]);
                Eigen::Matrix<double, 1, 6> jaco_ex;
                Eigen::Matrix3d I3x3;
                I3x3.setIdentity();

                jaco_ex.leftCols<3>() = ljm.transpose() * ril.transpose() * (Ri.transpose() * Rj - I3x3);
                jaco_ex.rightCols<3>() =
                   - ljm.transpose() * ril.transpose() * Ri.transpose() * Rj * ril * Utility::skewSymmetric(cp) +
                   ljm.transpose() * Utility::skewSymmetric(ril.transpose() *
                                      (Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi) - til));
                jacobian_ex.setZero();
                jacobian_ex.leftCols<6>() = sqrt_info * jaco_ex;
                jacobian_ex.rightCols<1>().setZero();
            }
        }

    return true;
    }

    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;
    double sqrt_info;
};

class LidarPlaneNormFactor : public ceres::SizedCostFunction<1, 7, 7>
{
public:
  LidarPlaneNormFactor() = delete;
  LidarPlaneNormFactor(Eigen::Vector3d curr_point_,
                       Eigen::Vector3d plane_unit_norm_,
                       double negative_OA_dot_norm_,
                       double sqrt_info_):
                       curr_point(curr_point_),
                       plane_unit_norm(plane_unit_norm_),
                       negative_OA_dot_norm(negative_OA_dot_norm_),
                       sqrt_info(sqrt_info_){}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

        Eigen::Vector3d t_w_curr(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q_w_curr(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d norm(plane_unit_norm.x(), plane_unit_norm.y(), plane_unit_norm.z());
        Eigen::Vector3d point_c(curr_point.x(), curr_point.y(), curr_point.z());

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qil(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_b = qil * point_c + til;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = q_w_curr * point_b + t_w_curr;

        double residual = norm.dot(point_w) + negative_OA_dot_norm;
        residuals[0] = sqrt_info * residual;

        if (jacobians) {
            Eigen::Matrix3d Rj = q_w_curr.toRotationMatrix();

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_j(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_j;

                jaco_j.leftCols<3>() = norm.transpose();
                jaco_j.rightCols<3>() =
                 - norm.transpose() * Rj * Utility::skewSymmetric(ril * point_c + til);

                jacobian_pose_j.setZero();
                jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
//                jacobian_pose_j.rightCols<1>().setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_ex(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_ex;

                jaco_ex.leftCols<3>() = norm.transpose() * Rj;
                jaco_ex.rightCols<3>() =
                 - norm.transpose() * Rj * ril * Utility::skewSymmetric(point_c);

                jacobian_ex.setZero();
                jacobian_ex.leftCols<6>() = sqrt_info * jaco_ex;
//                jacobian_ex.rightCols<1>().setZero();
            }
        }

    return true;
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double sqrt_info;
};

class LidarPointFactor : public ceres::SizedCostFunction<3, 7, 7>
{
public:
    LidarPointFactor() = delete;
    LidarPointFactor(Eigen::Vector3d curr_point_,
                     Eigen::Vector3d closed_point_,
                     double sqrt_info_):
                     curr_point(curr_point_),
                     closed_point(closed_point_),
                     sqrt_info(sqrt_info_) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond q_w_curr(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d t_w_curr(parameters[0][0], parameters[0][1], parameters[0][2]);

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qil(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        Eigen::Vector3d cp(curr_point.x(), curr_point.y(), curr_point.z());

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_b = qil * cp + til;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = q_w_curr * point_b + t_w_curr;

        residuals[0] = sqrt_info * (point_w.x() - closed_point.x());
        residuals[1] = sqrt_info * (point_w.y() - closed_point.y());
        residuals[2] = sqrt_info * (point_w.z() - closed_point.z());

        if (jacobians) {
          Eigen::Matrix3d Rj = q_w_curr.toRotationMatrix();
          Eigen::Matrix3d I3x3;
          I3x3.setIdentity();
          if (jacobians[0]) {
              Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);
              Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

              Eigen::Matrix3d tran = I3x3;
              Eigen::Matrix3d rot = - Rj * Utility::skewSymmetric(ril * cp + til);

              jaco_x.leftCols<3>() = tran.block<1,3>(0,0);
              jaco_y.leftCols<3>() = tran.block<1,3>(1,0);
              jaco_z.leftCols<3>() = tran.block<1,3>(2,0);
              jaco_x.rightCols<3>() = rot.block<1,3>(0,0);
              jaco_y.rightCols<3>() = rot.block<1,3>(1,0);
              jaco_z.rightCols<3>() = rot.block<1,3>(2,0);

              jacobian_pose_j.setZero();
              jacobian_pose_j.block<1,6>(0,0) = sqrt_info * jaco_x;
              jacobian_pose_j.block<1,6>(1,0) = sqrt_info * jaco_y;
              jacobian_pose_j.block<1,6>(2,0) = sqrt_info * jaco_z;
          }
          if (jacobians[1]) {
              Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_ex(jacobians[1]);
              Eigen::Matrix<double, 1, 6> jaco_exx, jaco_exy, jaco_exz;

              Eigen::Matrix3d ex_tran = Rj;
              Eigen::Matrix3d ex_rot = - Rj * ril * Utility::skewSymmetric(cp);
              jaco_exx.leftCols<3>() = ex_tran.block<1,3>(0,0);
              jaco_exy.leftCols<3>() = ex_tran.block<1,3>(1,0);
              jaco_exz.leftCols<3>() = ex_tran.block<1,3>(2,0);
              jaco_exx.rightCols<3>() = ex_rot.block<1,3>(0,0);
              jaco_exy.rightCols<3>() = ex_rot.block<1,3>(1,0);
              jaco_exz.rightCols<3>() = ex_rot.block<1,3>(2,0);

              jacobian_ex.setZero();
              jacobian_ex.block<1,6>(0,0) = sqrt_info * jaco_exx;
              jacobian_ex.block<1,6>(1,0) = sqrt_info * jaco_exy;
              jacobian_ex.block<1,6>(2,0) = sqrt_info * jaco_exz;
          }
        }

      return true;
    }

    Eigen::Vector3d curr_point, closed_point;
    double sqrt_info;
};
