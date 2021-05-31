#ifndef NCRL_LIO_FEATURE_MANAGER_H
#define NCRL_LIO_FEATURE_MANAGER_H
#include <list>
#include <algorithm>
#include <vector>
#include <numeric>

#include <ros/console.h>
#include <ros/assert.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include "ncrl_lio_parameters_fusion.h"
#include "tic_toc.h"
#include "ncrl_lio/ncrl_tf.h"
#include "factor/lidarFactorJaco.hpp"
#include "factor/integration_base.h"

typedef pcl::PointXYZI PointType;

// class for feature points handler in one frame
class FeaturePerSweep
{
public :
    FeaturePerSweep();
    void initial();

    void checkSize();

    // ==== INPUT from laserOdometry ====
    int id;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
    nav_msgs::Odometry laserOdom;
    // ==== INPUT from laserOdometry ====

    // === laserMapping ===
    // update with odometry
    int laserCloudCenWidth;
    int laserCloudCenHeight;
    int laserCloudCenDepth;

    // ====== optimization : all update in function UpdateAndFindMapFeaturePoint() ======
    int laserCloudCornerStackNum;
    int laserCloudSurfStackNum;

    // for voxel feature point last
    pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;

    // store index of pointcloud which build laserCloudCornerFromMap & laserCloudSurfFromMap
    int laserCloudValidInd[125];
    int laserCloudSurroundInd[125];
    int laserCloudValidNum;
    int laserCloudSurroundNum;

    IntegrationBase *pre_integration;

    ncrl_tf::Trans Tw2curr_I; // laserMapping
    ncrl_tf::Trans Tw2curr_L;
    ncrl_tf::Trans Tw2curr_W;

    // laserOdometry
    ncrl_tf::Trans Tinit2odom_L;
    ncrl_tf::Trans Tinit2odom_I;

    ncrl_tf::Trans Tinit2odom_st_L; // laserOdometry when initialStructure finish
    ncrl_tf::Trans Tinit2odom_st_I;
    ncrl_tf::Trans Tcurr2odom_I; // laserOdometry to laserMapping
    ncrl_tf::Trans Tcurr2odom_L;
};

class ncrl_lio_feature_manager
{
public:
    ncrl_lio_feature_manager();
    std::map<int, FeaturePerSweep> Feature_All_Map;

    // laser Mapping
    void downSizeFilter(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out, float res);

    // modifying
    void UpdateAndMaintainMapCube_All(SolverFlag flag);
    void UpdateState(SolverFlag flag);
    void UpdateAndFindMapFeaturePoint(FeaturePerSweep &Sweep_c);
//    void UpdateAndMaintainMapCube(FeaturePerSweep &Sweep_l, FeaturePerSweep &Sweep_c);
    void SetInputMapKdTree(pcl::PointCloud<PointType>::Ptr cornerMap, pcl::PointCloud<PointType>::Ptr surfaceMap);
    void pointAssociateToMap(PointType const *const pi, PointType *const po,
                             Eigen::Quaterniond q_w_curr, Eigen::Vector3d t_w_curr);
    bool KSearchMapKdTree(PointType pSel, int num, std::vector<int> &Ind, std::vector<float> &Dis, int flag);

    // initial Structure
    void solveGyroscopeBias(Eigen::Vector3d* Bgs);
    bool LinearAlignment(Eigen::Vector3d &g);
    bool SolveGravity(Eigen::Vector3d &g);
    void initSizeMaintainer();

    // construct cost function
    bool LidarPlaneNormHandler(PointType pointSel, Eigen::Vector3d &norm, double &negative_OA_dot_norm);
    bool LidarEdgeHandler(PointType pointSel, Eigen::Vector3d &point_a, Eigen::Vector3d &point_b);
    bool LidarF2fPlaneHandler(pcl::PointCloud<PointType>::Ptr laserCloudSurfLast, PointType pointSel,
                              int &closestPointInd, int &minPointInd2, int &minPointInd3);
    bool LidarF2fEdgeHandler(pcl::PointCloud<PointType>::Ptr laserCloudCornerLast, PointType pointSel,
                             int &closestPointInd, int &minPointInd2);

    bool LidarReloHandler(pcl::PointCloud<PointType>::Ptr input_cloud,
                          PointType pointSel, Eigen::Vector3d &closed_point);
    void SetReloKdTree(pcl::PointCloud<PointType>::Ptr input_cloud);

    void RANSAC_segmentation(pcl::PointCloud<PointType>::Ptr &foreground,
                             pcl::PointCloud<PointType>::Ptr &background);
    void background_extraction(pcl::PointCloud<PointType>::Ptr input_cloud,
                               pcl::PointCloud<PointType>::Ptr &foreground,
                               pcl::PointCloud<PointType>::Ptr &background);

    void pointL2I(Eigen::Vector3d p_i, PointType &p_o);
    void UpdateMap();

    bool initSystem = false;

    // test
    int count;
    double t_trans_end;

    // laserMapping
    const int laserCloudWidth = 21;
    const int laserCloudHeight = 21;
    const int laserCloudDepth = 11;
    const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;// 4851

    const float cubeSize = 50;
    const float cubeSizeHalf = cubeSize/2.0;

    // points in every Cube
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[4851];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[4851];

    // num of feature map which construct KD tree which determine do cost function or not
    int laserCloudCornerFromMapNum;
    int laserCloudSurfFromMapNum;

    // surround points in map to build tree
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;

    // store index of pointcloud which build laserCloudCornerFromMap & laserCloudSurfFromMap
    int laserCloudValidInd[125];
    int laserCloudSurroundInd[125];
    int laserCloudValidNum;
    int laserCloudSurroundNum;

    // all visualble cube points, pub for map result
    pcl::PointCloud<PointType>::Ptr laserCloudSurround;
    pcl::PointCloud<PointType>::Ptr laserCloudRegisted;

    ncrl_tf::Trans initGuess_I;
    ncrl_tf::Trans initGuess_L;

    pcl::PointCloud<PointType>::Ptr laserCloudStatic;

    // KDtree
    // for frame 2 frame odometry
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;
    // for frame 2 map odometry
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
};

#endif // NCRL_LIO_FEATURE_MANAGER_H
