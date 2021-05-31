#include "ncrl_lio/ncrl_lio_parameters_fusion.h"

int INIT_SIZE;

// accelerometer measurement noise standard deviation.
double ACC_N;
// gyroscope measurement noise standard deviation.
double GYR_N;
// accelerometer bias random work noise standard deviation.
double ACC_W;
// gyroscope bias random work noise standard deviation.
double GYR_W;

Eigen::Vector3d G{0.0, 0.0, 9.805};

double SOLVER_TIME = 0.04;
int NUM_ITERATIONS = 4;

double TD = 0.0;

std::string IMU_TOPIC;
std::string LIDAR_TOPIC;
std::string OUTPUT_PATH;

// flag control
int ENABLE_F2M_PLANE;
int ENABLE_F2M_EDGE;
int ENABLE_F2F_PLANE;
int ENABLE_F2F_EDGE;
int ENABLE_MARGINALIZATION;
int ENABLE_OPT_EXTRINSIC;
int ENABLE_PRIOR_FACTOR;
int ENABLE_HIGHSPEED_INITIALIZATION;
int ENABLE_REMOVE_BACKGROUND;

double lineRes = 0.3;
double planeRes = 0.6;
double BackgroundDis;
double ICP_Score = 0.8;
double KeyframeDistance = 1.0;
double LoopSearchRadius = 10.0;
double MinLoopDuration = 30.0;

// residual weight setting
double w_f2m_flat;
double w_f2m_corner;
double w_f2f_flat;
double w_f2f_corner;
double w_ext_tran;
double w_ext_rot;

Eigen::Vector3d ER_I2L{0, 0, 0};
Eigen::Vector3d ET_I2L{0, 0, 0};
Eigen::Vector3d ER_I2G{0, 0, 0};
Eigen::Vector3d ET_I2G{0, 0, 0};

ncrl_tf::Trans EXTRINSIC; // update in estimator_node
ncrl_tf::Trans WORLD2IMU; // update in estimator's initialStructure
ncrl_tf::Trans IMU2GT;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "/config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    if(!fsSettings.isOpened())
        ROS_FATAL("ERROR: Wrong path to settings");

    // setting path and topic name
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["lidar_topic"] >> LIDAR_TOPIC;
    fsSettings["output_path"] >> OUTPUT_PATH;

    INIT_SIZE = fsSettings["INITIAL_STRUCTURE_SIZE"];

    // flag control
    ENABLE_F2M_PLANE = fsSettings["enable_F2M_PLANE"];
    ENABLE_F2M_EDGE = fsSettings["enable_F2M_EDGE"];
    ENABLE_F2F_PLANE = fsSettings["enable_F2F_PLANE"];
    ENABLE_F2F_EDGE = fsSettings["enable_F2F_EDGE"];
    ENABLE_MARGINALIZATION = fsSettings["enable_MARGINALIZATION"];
    ENABLE_OPT_EXTRINSIC = fsSettings["enable_OPT_EXTRINSIC"];
    ENABLE_PRIOR_FACTOR = fsSettings["enable_PRIOR_FACTOR"];
    ENABLE_HIGHSPEED_INITIALIZATION = fsSettings["enable_HIGHSPEED_INITIALIZATION"];
    ENABLE_REMOVE_BACKGROUND = fsSettings["enable_REMOVE_BACKGROUND"];

    if (!ENABLE_F2F_PLANE && !ENABLE_F2F_EDGE)
      ENABLE_MARGINALIZATION = 0;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];

    // input as degree and meter
    ER_I2L.x() = fsSettings["EI2L_RX"];
    ER_I2L.y() = fsSettings["EI2L_RY"];
    ER_I2L.z() = fsSettings["EI2L_RZ"];
    ER_I2L = deg2rad(ER_I2L);
    ET_I2L.x() = fsSettings["EI2L_TX"];
    ET_I2L.y() = fsSettings["EI2L_TY"];
    ET_I2L.z() = fsSettings["EI2L_TZ"];

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    lineRes = fsSettings["lineRes"];
    planeRes = fsSettings["planeRes"];
    BackgroundDis = fsSettings["BackgroundDis"];

    ICP_Score = fsSettings["ICP_Score"];
    KeyframeDistance = fsSettings["KeyframeDistance"];
    LoopSearchRadius = fsSettings["LoopSearchRadius"];
    MinLoopDuration = fsSettings["MinLoopDuration"];

    w_f2m_flat = fsSettings["w_f2m_flat"];
    w_f2m_corner = fsSettings["w_f2m_corner"];
    w_f2f_flat = fsSettings["w_f2f_flat"];
    w_f2f_corner = fsSettings["w_f2f_corner"];
    w_ext_tran = fsSettings["w_ext_tran"];
    w_ext_rot = fsSettings["w_ext_rot"];

    TD = fsSettings["td"];

    ER_I2G.x() = fsSettings["EXGT_RX"];
    ER_I2G.y() = fsSettings["EXGT_RY"];
    ER_I2G.z() = fsSettings["EXGT_RZ"];
    ER_I2G = deg2rad(ER_I2G);
    ET_I2G.x() = fsSettings["EXGT_TX"];
    ET_I2G.y() = fsSettings["EXGT_TY"];
    ET_I2G.z() = fsSettings["EXGT_TZ"];

    fsSettings.release();
}
