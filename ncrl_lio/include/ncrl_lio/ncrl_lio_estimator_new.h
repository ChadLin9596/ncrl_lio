#ifndef NCRL_LIO_ESTIMATOR_NEW_H
#define NCRL_LIO_ESTIMATOR_NEW_H
// ros msg
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

// ceres lib
#include <ceres/ceres.h>

#include "utility/utility.h"
#include "tic_toc.h"
#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "ncrl_lio_parameters_fusion.h"
#include "ncrl_lio_feature_manager_fusion.h"
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include "factor/imu_factor.h"
#include "factor/extrinsicFactor.hpp"
#include "aloam/common_modify.h"

class ncrl_lio_estimator_new
{
public:
    ncrl_lio_estimator_new();
    // desire
    void processLIO(FeaturePerSweep &feature_msg, int lidar_id);
    void showTimeCost();
    void maintainSize();

    // ======= sliding wondow =======
    void optimization();
    void vector2double();
    void double2vector();

    // imu use
    void processImu(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    bool initialStructure();
    bool LidarImuAlign();
    bool checkIMUexcitation();

    // Marginalization
    void marginalization();

    // ======= flag define =======
    SolverFlag solver_flag = INITIAL;

    // Marginalization setting
    MarginalizationFlag  marginalization_flag;
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    queue<ResidualBlockInfo *> f2f_block;

    // initial structure
    int frame_count = 0;

    // Object of input data
    ncrl_lio_feature_manager *Feature_All;

    // eigen state
    Eigen::Vector3d Ps[SWEEP_SIZE + 1];
    Eigen::Quaterniond Qs[SWEEP_SIZE + 1];
    Eigen::Vector3d Vs[SWEEP_SIZE + 1];
    Eigen::Vector3d Bas[SWEEP_SIZE + 1];
    Eigen::Vector3d Bgs[SWEEP_SIZE + 1];
    Eigen::Quaterniond qil[1];
    Eigen::Vector3d til[1];
    Eigen::Quaterniond w2init[1];

    // ceres state
    double para_Pose[SWEEP_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[SWEEP_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Ex_Pose[1][SIZE_POSE];
    double para_W2Init[1][SIZE_ROTATION];

    // time cost
    double t_whole = -1;
    double t_cereSolver = -1;
    double t_whole_max = -1;
    double t_whole_sum = 0;
    int t_count = 0;

    // process imu varibale
    bool first_imu = false;
    IntegrationBase *tmp_pre_integration;
    Vector3d acc_0, gyr_0; // imu msg last time
    ncrl_tf::Point g;

    // cost
    struct Cost
    {
        double prev;
        double after;
    };
    Cost C_imu;
    Cost C_F2M_E;
    Cost C_F2M_P;
    Cost C_F2F_E;
    Cost C_F2F_P;
    Cost C_Marginal;
};


#endif // NCRL_LIO_ESTIMATOR_NEW_H
