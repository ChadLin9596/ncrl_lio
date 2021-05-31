#ifndef NCRL_TF_H
#define NCRL_TF_H
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;
typedef string FrameID;

class ncrl_tf
{
public:
    ncrl_tf();

    struct Point{
      FrameID frame;
      Eigen::Vector3d point;
    };

    struct Trans{
      FrameID start_frame;
      FrameID end_frame;
      Eigen::Quaterniond q;
      Eigen::Vector3d v;
    };

#if 1
    template <typename Derived>
    static void changeState(Eigen::MatrixBase<Derived> &state){
      state.setZero();
    }

    template <typename Derived>
    static void showState(Eigen::MatrixBase<Derived> &state){
      std::cout << state.transpose() << std::endl;
    }

    template <typename Derived>
    static void showQuaternionState(Eigen::QuaternionBase<Derived> &state){
      std::cout << state.coeffs().transpose() << std::endl;
    }
#endif
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Euler2Q(Eigen::MatrixBase<Derived> &euler){
      typedef typename Derived::Scalar Scalar_t;
      Eigen::Quaternion<Scalar_t> q;
      q = Eigen::AngleAxis<Scalar_t>(euler.z(), Eigen::MatrixBase<Derived>::UnitZ()) *
          Eigen::AngleAxis<Scalar_t>(euler.y(), Eigen::MatrixBase<Derived>::UnitY()) *
          Eigen::AngleAxis<Scalar_t>(euler.x(), Eigen::MatrixBase<Derived>::UnitX());
      return q;
    }

    static Eigen::Vector3d Q2Euler(Eigen::Quaterniond &Q){
      Eigen::Vector3d euler = Q.toRotationMatrix().eulerAngles(2, 1, 0);
      Eigen::Vector3d result(euler(2),euler(1),euler(0));
      return result;
    }

    static void setTransFrame(Trans &tf, FrameID sf, FrameID ef){
      if (sf != ef){
        tf.start_frame = sf;
        tf.end_frame = ef;
      } else {
        std::cout << "Origin frame from " << sf << " -> " << ef << "\nPlease reset frame" << std::endl;
      }
    }

    static void setPointFrame(Point &p, FrameID f){
      p.frame = f;
    }

    static void setTrans(Trans &tf, Eigen::Quaterniond q, Eigen::Vector3d v){
      tf.q = q;
      tf.v = v;
    }

    static void setPoint(Point &p, Eigen::Vector3d v){
      p.point = v;
    }

    template <typename Derived, typename OtherDerived>
    static void setTfTrans(tf::Transform &TF, Eigen::QuaternionBase<Derived>& Q, Eigen::MatrixBase<OtherDerived>& T){
      tf::Quaternion tf_q(Q.x(), Q.y(), Q.z(), Q.w());
      tf::Vector3 tf_v(T(0), T(1), T(2));
      TF.setRotation(tf_q);
      TF.setOrigin(tf_v);
    }

    static bool TransPoint(Trans t, Point &p){
      if (p.frame == t.start_frame && p.frame != t.end_frame){
        p.point = t.q.inverse() * (p.point - t.v);
        p.frame = t.end_frame;
        return true;
      } else if(p.frame == t.end_frame && p.frame != t.start_frame){
        p.point = t.q * p.point + t.v;
        p.frame = t.start_frame;
        return true;
      }
      std::cout << "point frame " << p.frame << " is wrong\n"
                << "odometry from " << t.start_frame
                << " to " << t.end_frame << std::endl;
      return false;
    }

    // odom_i : input, odom_o : output
    static bool TransOdometry(const Trans &t, Trans &odom_i, Trans &odom_o){
      if ((odom_i.end_frame == t.start_frame && odom_o.end_frame == t.end_frame) ||
          (odom_i.start_frame == t.start_frame && odom_o.start_frame == t.end_frame)){
        odom_o.q = t.q.inverse() * odom_i.q * t.q;
        odom_o.v = t.q.inverse() * (odom_i.q * t.v + odom_i.v - t.v);
      } else if ((odom_i.end_frame == t.end_frame && odom_o.end_frame == t.start_frame) ||
                 (odom_i.start_frame == t.end_frame && odom_o.start_frame == t.start_frame)){
        odom_o.q = t.q * odom_i.q * t.q.inverse();
        odom_o.v = t.v + t.q * odom_i.v - odom_o.q * t.v;
      } else{
        std::cout << "odometry frame is wrong\n"
                  << "t : " << t.start_frame << " -> " << t.end_frame
                  << "odom_i : " << odom_i.start_frame << " -> " << odom_i.end_frame
                  << "odom_o : " << odom_o.start_frame << " -> " << odom_o.end_frame << std::endl;
        return false;
      }
      return true;
    }

    static bool deltaTrans(Trans &Tij, Trans& Ti, Trans& Tj){
      if (Ti.start_frame == Tj.start_frame){
        Tij.q = Ti.q.inverse() * Tj.q;
        Tij.v = Ti.q.inverse() * (Tj.v - Ti.v);
        setTransFrame(Tij, Ti.end_frame, Tj.end_frame);
      } else if (Ti.end_frame == Tj.end_frame){
        Tij.q = Ti.q * Tj.q.inverse();
        Tij.v = Ti.v - Tij.q * Tj.v;
        setTransFrame(Tij, Ti.start_frame, Tj.start_frame);
      } else {
        std::cout << "Delta transform is wrong" << std::endl;
        return false;
      }
      return true;
    }

    static bool accumTrans(Trans &Tj, Trans& Ti, Trans Tij){
      if (Tij.start_frame == Ti.end_frame){
        Tj.q = Ti.q * Tij.q;
        Tj.v = Ti.v + Ti.q * Tij.v;
        setTransFrame(Tj, Ti.start_frame, Tij.end_frame);
      } else {
        std::cout << "frame in accumulate transframe is wrong" << std::endl;
        return false;
      }
      return true;
    }
};
#endif
