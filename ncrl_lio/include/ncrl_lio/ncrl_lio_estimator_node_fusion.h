#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "ncrl_lio_estimator_new.h"
#include "ncrl_lio_parameters_fusion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef pcl::PointXYZI PointType;
typedef std::pair<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> FeatureType;

typedef std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                              std::pair<nav_msgs::OdometryConstPtr,
                                        std::pair<sensor_msgs::PointCloud2ConstPtr, FeatureType>>>> CombinedData;
