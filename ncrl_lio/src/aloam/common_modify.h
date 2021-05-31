#pragma once

#include <cmath>

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;
struct PointXYZIQT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    double qw;
    double qx;
    double qy;
    double qz;
    double time;
    int loopID;
    double loop_tx;
    double loop_ty;
    double loop_tz;
    double loop_yaw;
    double var;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIQT,
                                   (double, x, x) (double, y, y)
                                   (double, z, z) (double, intensity, intensity)
                                   (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
                                   (double, time, time) (int, loopID, loopID)
                                   (double, loop_tx, loop_tx) (double, loop_ty, loop_ty) (double, loop_tz, loop_tz)
                                   (double, loop_yaw, loop_yaw) (double, var, var)
)
typedef PointXYZIQT  PointTypePose;

inline Eigen::Vector3d rad2deg(Eigen::Vector3d radians)
{
  return radians * 180.0 / M_PI;
}

inline Eigen::Vector3d deg2rad(Eigen::Vector3d degrees)
{
  return degrees * M_PI / 180.0;
}

inline Eigen::Vector3d Q2RPY(Eigen::Quaterniond Q)
{
  Eigen::Vector3d RPY(0, 0, 0);
  RPY[0] = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), (1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y())));
  RPY[1] = asin(2 * (Q.w() * Q.y() + Q.z() * Q.x()));
  RPY[2] = atan2(2 * (Q.w() * Q.z() + Q.x() * Q.y()), (1 - 2 * (Q.y() * Q.y() + Q.z() * Q.z())));
  return RPY;
}

// rotation matrix to yaw pitch row (degree)
inline Eigen::Vector3f R2ypr(const Eigen::Matrix3f &R)
{
    Eigen::Vector3f n = R.col(0);
    Eigen::Vector3f o = R.col(1);
    Eigen::Vector3f a = R.col(2);

    Eigen::Vector3f ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

//    return ypr / M_PI * 180.0;
    return ypr;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(ypr(0)), -sin(ypr(0)), 0,
        sin(ypr(0)), cos(ypr(0)), 0,
        0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(ypr(1)), 0., sin(ypr(1)),
        0., 1., 0.,
        -sin(ypr(1)), 0., cos(ypr(1));

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(ypr(2)), -sin(ypr(2)),
        0., sin(ypr(2)), cos(ypr(2));

    return Rz * Ry * Rx;
}
