#include <cmath>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "ncrl_lio/tic_toc.h"
#include <std_msgs/Bool.h>

#include <ceres/ceres.h>
#include "factor/lidarFactor.hpp"

//use mocap for unwrap or VIO
//#define use_mocap
typedef pcl::PointXYZI PointType;

using std::atan2;
using std::cos;
using std::sin;

//value to determine how much period in a sweep
const int cluster_size = 6;
const double SCAN_PERIOD = 0.1;
const double DISTANCE_SQ_THRESHOLD = 25;
const double NEARBY_SCAN = 2.5;

bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubUndistort;
ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;

double MINIMUM_RANGE = 0.1;
double Start_Frame_Time = -1;
double delay_t = 0;
std::map<double, std::vector<double>> pose_map;
std::queue<sensor_msgs::PointCloud2ConstPtr> cloudBuf;
std::mutex mBuf;
double t_sum = 0;
int cloud_count = 0;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void TransformToStart(PointType const *const pi, PointType *const po, Eigen::Vector3d t_w_curr, Eigen::Quaterniond q_w_curr)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

void TransformToEnd(PointType const *const pi, PointType *const po, Eigen::Vector3d t_w_curr, Eigen::Quaterniond q_w_curr)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_end = q_w_curr.inverse() * (point_curr - t_w_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();
    po->intensity = pi->intensity;
}

void replaceIntensity(pcl::PointCloud<PointType>::Ptr &laserCloudIn){
    bool halfPassed_ = false;
    float startOri_ = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    float endOri_ = -atan2(laserCloudIn->points[laserCloudIn->size() - 1].y,
                           laserCloudIn->points[laserCloudIn->size() - 1].x) + 2 * M_PI;

    if (endOri_ - startOri_ > 3 * M_PI)
        endOri_ -= 2 * M_PI;
    else if (endOri_ - startOri_ < M_PI)
        endOri_ += 2 * M_PI;

    for (size_t i = 0; i < laserCloudIn->size(); i++)
    {
        PointType point;
        point.x = laserCloudIn->points[i].x;
        point.y = laserCloudIn->points[i].y;
        point.z = laserCloudIn->points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16){
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32){
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64){
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                continue;
            }
        }
        else{
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        float ori = -atan2(point.y, point.x);

        if (!halfPassed_)
        {
            if (ori < startOri_ - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri_ + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri_ > M_PI)
            {
                halfPassed_ = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri_ - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri_ + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri_) / (endOri_ - startOri_);
        laserCloudIn->points[i].intensity = scanID + SCAN_PERIOD * relTime;
    }
}

void Feature_Extraction(pcl::PointCloud<PointType>::Ptr laserCloudIn, std_msgs::Header header,
                        pcl::PointCloud<PointType> &cornerPointsSharp, pcl::PointCloud<PointType> &cornerPointsLessSharp,
                        pcl::PointCloud<PointType> &surfPointsFlat, pcl::PointCloud<PointType> &surfPointsLessFlat){
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (size_t i = 0; i < laserCloudIn->size(); i++)
        laserCloudScans[int(laserCloudIn->points[i].intensity)].push_back(laserCloudIn->points[i]);

    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    for (size_t i = 5; i < laserCloud->size() - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x
                    + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                    + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x
                    + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y
                    + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                    + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y
                    + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z
                    + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                    + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z
                    + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }

    //publish features
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = header.stamp;
    laserCloudOutMsg.header.frame_id = "/velodyne";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/velodyne";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/velodyne";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = header.stamp;
    surfPointsFlat2.header.frame_id = "/velodyne";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = header.stamp;
    surfPointsLessFlat2.header.frame_id = "/velodyne";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
}

void pose_cb (const nav_msgs::OdometryConstPtr& msg) {
    if (!systemInited)
        systemInited = true;

    mBuf.lock();
    // pose from LIO is in IMU frame
    double t = msg->header.stamp.toSec();
    Eigen::Vector3d P_Imu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond Q_Imu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);

    Eigen::Quaterniond I2L_Q(1, 0, 0, 0);
//    Eigen::Vector3d I2L_T(0, 0.08, -0.12);
    Eigen::Vector3d I2L_T(0, 0, 0.1);
    // transform to world -> lidar
    Eigen::Quaterniond Q_Lidar = Q_Imu * I2L_Q;
    Eigen::Vector3d P_Lidar = Q_Imu * I2L_T + P_Imu;

    if (1) {
        std::vector<double> pose{P_Lidar.x(), P_Lidar.y(), P_Lidar.z(),
                                 Q_Lidar.w(), Q_Lidar.x(), Q_Lidar.y(), Q_Lidar.z()};
        pose_map[t] = pose;
    }
    mBuf.unlock();
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    mBuf.lock();
    cloudBuf.push(laserCloudMsg);
    mBuf.unlock();
}

bool getMeasurement(std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> &data) {
    bool r = false;
    if (!cloudBuf.empty() && !pose_map.empty() &&
        cloudBuf.front()->header.stamp.toSec() + delay_t <= pose_map.rbegin()->first) {
        r = true;
        data.clear();

        // cluster here
        int cnt = 0;
        std::map<double, std::vector<double>>::iterator iter = pose_map.begin();
        std::map<double, std::vector<double>>::iterator iter_closest = pose_map.begin();

        // assure result is cluster_size
        while (cnt != cluster_size){
            double desire_t = cloudBuf.front()->header.stamp.toSec() + delay_t
                            - SCAN_PERIOD + cnt * SCAN_PERIOD / cluster_size
                            + 0.5 * SCAN_PERIOD / cluster_size;
            double t_diff = desire_t - iter->first;

            // extract the candidate
            if (fabs(desire_t - iter_closest->first) > fabs(t_diff))
                iter_closest = iter;

            // need to check the missing data problem
            if (t_diff < 0){
                Eigen::Quaterniond Q_I2L(1, 0, 0, 0);
                Eigen::Vector3d V_I2L(0, 0.08, -0.12);

                Eigen::Quaterniond Q_(iter_closest->second[3],
                                      iter_closest->second[4],
                                      iter_closest->second[5],
                                      iter_closest->second[6]);
                Eigen::Vector3d V_(iter_closest->second[0],
                                   iter_closest->second[1],
                                   iter_closest->second[2]);
                // transform to imu
                V_ = V_ + Q_ * V_I2L;
                Q_ = Q_ * Q_I2L;

                data.push_back(std::make_pair(Q_, V_));
                cnt++;
            }
            iter++;
        }

        // maintain pose_map
        pose_map.erase(pose_map.begin(), iter_closest);
    }
    return r;
}

void handleDistortion(pcl::PointCloud<PointType>::Ptr &laserCloudIn,
                      std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> integration){
    Eigen::Quaterniond q_w_end = integration[cluster_size - 1].first;
    Eigen::Vector3d t_w_end = integration[cluster_size - 1].second;
    for (size_t i = 0; i < laserCloudIn->points.size(); i++){
        double t_point = laserCloudIn->points[i].intensity - (int)laserCloudIn->points[i].intensity;
        // 0 - cluster_size
        int ind = t_point / (SCAN_PERIOD / cluster_size);
        if (ind < cluster_size){
            Eigen::Quaterniond q_w_start = integration[ind].first;
            Eigen::Vector3d t_w_start = integration[ind].second;
            // transform to start
            TransformToStart(&laserCloudIn->points[i], &laserCloudIn->points[i],
                             t_w_start, q_w_start);
            // transform to end
            TransformToEnd(&laserCloudIn->points[i], &laserCloudIn->points[i],
                           t_w_end, q_w_end);
        }
    }
}

void process() {
    while(true){
        if(systemInited){
            std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> integration;
            if (getMeasurement(integration)){
                ROS_WARN("Starting handling distortion!");

                // original scanRegistration
                TicToc t_whole;
                pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
                std_msgs::Header header;
                mBuf.lock();
                pcl::fromROSMsg(*cloudBuf.front(), *laserCloud);
                header = cloudBuf.front()->header;
                cloudBuf.pop();
                mBuf.unlock();

                // pre-process
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indices);
                removeClosedPointCloud(*laserCloud, *laserCloud, MINIMUM_RANGE);
                replaceIntensity(laserCloud);
                handleDistortion(laserCloud, integration);

                // scanRegistration
                TicToc t_feature;
                pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
                Feature_Extraction(laserCloud, header, *cornerPointsSharp, *cornerPointsLessSharp, *surfPointsFlat, *surfPointsLessFlat);
                ROS_INFO("t_feature: %f ms", t_feature.toc());

                Start_Frame_Time = header.stamp.toSec();
                cloud_count++;
                t_sum += t_whole.toc();
                ROS_INFO("avg t_whole: %f ms", t_sum/cloud_count);
            }
        } else {
            if (!cloudBuf.empty()){
                // original scanRegistration
                ROS_WARN("Publish raw pointcloud!");
                TicToc t_whole;
                pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
                std_msgs::Header header;
                mBuf.lock();
                pcl::fromROSMsg(*cloudBuf.front(), *laserCloud);
                header = cloudBuf.front()->header;
                cloudBuf.pop();
                mBuf.unlock();

                // pre-process
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indices);
                removeClosedPointCloud(*laserCloud, *laserCloud, MINIMUM_RANGE);
                replaceIntensity(laserCloud);

                // scanRegistration
                TicToc t_feature;
                pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
                Feature_Extraction(laserCloud, header, *cornerPointsSharp, *cornerPointsLessSharp, *surfPointsFlat, *surfPointsLessFlat);
                ROS_INFO("t_feature: %f ms", t_feature.toc());

                Start_Frame_Time = header.stamp.toSec();
                cloud_count++;
                t_sum += t_whole.toc();
                ROS_INFO("avg t_whole: %f ms", t_sum/cloud_count);
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ncrl_cloud_regist");
    ros::NodeHandle nh;
    nh.param<int>("scan_line", N_SCANS, 16);
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);
    nh.param<double>("time_delay", delay_t, 0.0);
    printf("scan line number %d \n", N_SCANS);
    ROS_INFO_STREAM("time delay is " << delay_t);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry> ("/estimator/imu_propagate", 100, pose_cb);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/scanRegistration/velodyne_cloud_2", 100);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/scanRegistration/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/scanRegistration/laser_cloud_less_sharp", 100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/scanRegistration/laser_cloud_flat", 100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/scanRegistration/laser_cloud_less_flat", 100);

    std::thread regist_process{process};
    ros::spin();

    return 0;
}
