#ifndef NCRL_LIO_PARAMETERS_H
#define NCRL_LIO_PARAMETERS_H
#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <stdio.h>
#include "ncrl_tf.h"
#include "aloam/common_modify.h"

#define SHOWTIME true

using namespace std;
// setting path and topic name
extern std::string IMU_TOPIC;
extern std::string LIDAR_TOPIC;
extern std::string OUTPUT_PATH;

const int SWEEP_SIZE = 2;
const int NUM_OF_LIDAR = 1;

extern int INIT_SIZE;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;

extern double TD;

// flag control
extern int ENABLE_F2M_PLANE;
extern int ENABLE_F2M_EDGE;
extern int ENABLE_F2F_PLANE;
extern int ENABLE_F2F_EDGE;
extern int ENABLE_MARGINALIZATION;
extern int ENABLE_OPT_EXTRINSIC;
extern int ENABLE_PRIOR_FACTOR;
extern int ENABLE_HIGHSPEED_INITIALIZATION;
extern int ENABLE_REMOVE_BACKGROUND;

// parameter for Maintaining map
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

// parameter for voxle filter setting
extern double lineRes;
extern double planeRes;
extern double BackgroundDis;

// parameter for loop closure
extern double ICP_Score;
extern double KeyframeDistance;
extern double LoopSearchRadius;
extern double MinLoopDuration;

// residual weight setting
extern double w_f2m_flat;
extern double w_f2m_corner;
extern double w_f2f_flat;
extern double w_f2f_corner;
extern double w_ext_tran;
extern double w_ext_rot;

// extrinsic state setting
extern Eigen::Vector3d ER_I2L;
extern Eigen::Vector3d ET_I2L;
extern Eigen::Vector3d ER_I2G;
extern Eigen::Vector3d ET_I2G;

extern ncrl_tf::Trans EXTRINSIC;
extern ncrl_tf::Trans WORLD2IMU; // WORLD -> ORIGIN -> IMU_INIT
extern ncrl_tf::Trans IMU2GT;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_ROTATION = 4,
    SIZE_POSITION = 3
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

enum FeatureFlag
{
    CORNER = 0,
    SURFACE = 1
};

enum SolverFlag
{
    INITIAL,
    NON_LINEAR
};

enum MarginalizationFlag
{
    MARGIN_OLD = 0,
    MARGIN_NEW = 1
};

#endif // NCRL_LIO_PARAMETERS_H
