#include "ncrl_lio/ncrl_lio_feature_manager_fusion.h"

FeaturePerSweep::FeaturePerSweep():
  laserCloudCornerLast(new pcl::PointCloud<PointType>()),
  laserCloudSurfLast(new pcl::PointCloud<PointType>()),
  laserCloudFullRes(new pcl::PointCloud<PointType>()),
  laserCloudCornerStack(new pcl::PointCloud<PointType>()),
  laserCloudSurfStack(new pcl::PointCloud<PointType>())
{
    initial();
}
void FeaturePerSweep::initial(){
    id = -1;
    // laser Mapping
    laserCloudCenWidth = 10;
    laserCloudCenHeight = 10;
    laserCloudCenDepth = 5;

    // laserMapping
    laserCloudCornerStackNum = -1;
    laserCloudSurfStackNum = -1;
    laserCloudValidNum = -1;
    laserCloudSurroundNum = -1;

    ncrl_tf::setTransFrame(Tw2curr_I, "IMU_INIT", "IMU");
    ncrl_tf::setTransFrame(Tw2curr_L, "LIDAR_INIT", "LIDAR");
    ncrl_tf::setTransFrame(Tw2curr_W, "WORLD", "IMU");

    ncrl_tf::setTransFrame(Tinit2odom_st_I, "IMU_ODOM_START", "IMU");
    ncrl_tf::setTransFrame(Tinit2odom_st_L, "LIDAR_ODOM_START", "LIDAR");
    ncrl_tf::setTransFrame(Tcurr2odom_I, "IMU_INIT", "IMU_ODOM_START");
    ncrl_tf::setTransFrame(Tcurr2odom_L, "LIDAR_INIT", "LIDAR_ODOM_START");

    ncrl_tf::setTransFrame(Tinit2odom_L, "LIDAR_ODOM", "LIDAR");
    ncrl_tf::setTransFrame(Tinit2odom_I, "IMU_ODOM", "IMU");
    Eigen::Quaterniond identityQ(1,0,0,0);
    Eigen::Vector3d zeroV(0,0,0);
    ncrl_tf::setTrans(Tw2curr_I, identityQ, zeroV);
    ncrl_tf::setTrans(Tw2curr_L, identityQ, zeroV);
    ncrl_tf::setTrans(Tw2curr_W, identityQ, zeroV);
    ncrl_tf::setTrans(Tcurr2odom_L, identityQ, zeroV);
    ncrl_tf::setTrans(Tcurr2odom_I, identityQ, zeroV);
    ncrl_tf::setTrans(Tinit2odom_L, identityQ, zeroV);
    ncrl_tf::setTrans(Tinit2odom_I, identityQ, zeroV);
    ncrl_tf::setTrans(Tinit2odom_st_I, identityQ, zeroV);
    ncrl_tf::setTrans(Tinit2odom_st_L, identityQ, zeroV);
}

void FeaturePerSweep::checkSize(){
    if (laserCloudCornerLast->points.size() != 0 && laserCloudSurfLast->points.size() != 0 && laserCloudFullRes->points.size() != 0){
        std::cout << "laserOdometry input size : " << laserCloudCornerLast->points.size() << " "
                                                   << laserCloudSurfLast->points.size() << " "
                                                   << laserCloudFullRes->points.size() << std::endl;
    } else
        ROS_WARN("LASERODOMETRY IS WRONG");
}

ncrl_lio_feature_manager::ncrl_lio_feature_manager():
  laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
  laserCloudSurfFromMap(new pcl::PointCloud<PointType>()),
  laserCloudSurround(new pcl::PointCloud<PointType>()),
  laserCloudRegisted(new pcl::PointCloud<PointType>()),
  laserCloudStatic(new pcl::PointCloud<PointType>()),
  kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>()),
  kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>()),
  kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()),
  kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>())
{
    Feature_All_Map.clear();
    t_trans_end = -1;
    count = 0;

    // laserMapping
    laserCloudCornerFromMapNum = 0;
    laserCloudSurfFromMapNum = 0;
    laserCloudValidNum = 0;
    laserCloudSurroundNum = 0;

    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    }

    Eigen::Quaterniond identityQ(1,0,0,0);
    Eigen::Vector3d zeroV(0,0,0);
    ncrl_tf::setTransFrame(initGuess_I, "IMU_ODOM", "IMU_ODOM_START");
    ncrl_tf::setTransFrame(initGuess_L, "LIDAR_ODOM", "LIDAR_ODOM_START");
    ncrl_tf::setTrans(initGuess_I, identityQ, zeroV);
    ncrl_tf::setTrans(initGuess_L, identityQ, zeroV);
}

// ==================================================
void ncrl_lio_feature_manager::downSizeFilter(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out, float res){
  pcl::VoxelGrid<PointType> vox;
  vox.setInputCloud(cloud_in);
  vox.setLeafSize(res, res, res);
  vox.filter(*cloud_out);
}

void ncrl_lio_feature_manager::UpdateAndMaintainMapCube_All(SolverFlag flag){
//    int solved_iter = Feature_All_Map.size() - 1;
    // output
    // laserCloudCornerFromMap, laserCloudCornerFromMapNum,
    // laserCloudSurfFromMap, laserCloudSurfFromMapNum
    // laserCloudCornerStack, laserCloudCornerStackNum
    // laserCloudSurfStack, laserCloudSurfStackNum
    std::map<int, FeaturePerSweep>::iterator frame_i;
    int count =  0;
    for (frame_i = Feature_All_Map.begin(); frame_i != Feature_All_Map.end(); frame_i++){
        if (flag == NON_LINEAR){
            if(count == (int)Feature_All_Map.size() - 1){
                UpdateAndFindMapFeaturePoint(frame_i->second);
            }
        }
        // downsize last feature point into stack and update num
        frame_i->second.laserCloudCornerStack->clear();
        frame_i->second.laserCloudSurfStack->clear();

        downSizeFilter(frame_i->second.laserCloudCornerLast, frame_i->second.laserCloudCornerStack, lineRes);
        downSizeFilter(frame_i->second.laserCloudSurfLast, frame_i->second.laserCloudSurfStack, planeRes);
        frame_i->second.laserCloudCornerStackNum = frame_i->second.laserCloudCornerStack->points.size();
        frame_i->second.laserCloudSurfStackNum = frame_i->second.laserCloudSurfStack->points.size();

        for(int i = 0; i < frame_i->second.laserCloudCornerStackNum; i++){
            frame_i->second.laserCloudCornerStack->points[i].intensity -= 1000;
        }
//        for(int i = 0; i < frame_i->second.laserCloudSurfStackNum; i++){
//            ROS_WARN("intensity: %f", frame_i->second.laserCloudSurfStack->points[i].intensity);
//            frame_i->second.laserCloudSurfStack->points[i].intensity = 1;
//        }
        count++;
    }
}

// only update the last one in feature_all
void ncrl_lio_feature_manager::UpdateState(SolverFlag flag){
    std::map<int, FeaturePerSweep>::iterator frame_i, frame_j;
    int count = 0;
    for (frame_j = Feature_All_Map.begin(); frame_j != Feature_All_Map.end(); frame_j++){
        if (flag == NON_LINEAR){
            // only update the latest window
            if (count == (int)Feature_All_Map.size() - 1){
                frame_i = std::prev(frame_j);
                frame_j->second.Tcurr2odom_I = frame_i->second.Tcurr2odom_I;
                frame_j->second.Tcurr2odom_L = frame_i->second.Tcurr2odom_L;
                frame_j->second.laserCloudCenWidth = frame_i->second.laserCloudCenWidth;
                frame_j->second.laserCloudCenHeight = frame_i->second.laserCloudCenHeight;
                frame_j->second.laserCloudCenDepth = frame_i->second.laserCloudCenDepth;

                if(!ncrl_tf::deltaTrans(frame_j->second.Tinit2odom_st_I, initGuess_I, frame_j->second.Tinit2odom_I))
                    ROS_WARN("Calibriate initial guess from imu_odom_start to imu fail");
                if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_I, frame_j->second.Tcurr2odom_I, frame_j->second.Tinit2odom_st_I))
                    ROS_WARN("update state with new odom on IMU frame is fail");
                // lidar frame odom initial guess
                if(!ncrl_tf::deltaTrans(frame_j->second.Tinit2odom_st_L, initGuess_L, frame_j->second.Tinit2odom_L))
                    ROS_WARN("Calibriate initial guess from lidar_odom_start to lidar fail");
                if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_L, frame_j->second.Tcurr2odom_L, frame_j->second.Tinit2odom_st_L))
                    ROS_WARN("update state with new odom on IMU frame is fail");
                if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_W, WORLD2IMU, frame_j->second.Tw2curr_I))
                    ROS_WARN("update state with new odom on WORLD frame is fail");
            }
        } else {
            if(!ncrl_tf::deltaTrans(frame_j->second.Tinit2odom_st_I, initGuess_I, frame_j->second.Tinit2odom_I))
                ROS_WARN("Calibriate initial guess from imu_odom_start to imu fail");
            if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_I, frame_j->second.Tcurr2odom_I, frame_j->second.Tinit2odom_st_I))
                ROS_WARN("update state with new odom on IMU frame is fail");
            // lidar frame odom initial guess
            if(!ncrl_tf::deltaTrans(frame_j->second.Tinit2odom_st_L, initGuess_L, frame_j->second.Tinit2odom_L))
                ROS_WARN("Calibriate initial guess from lidar_odom_start to lidar fail");
            if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_L, frame_j->second.Tcurr2odom_L, frame_j->second.Tinit2odom_st_L))
                ROS_WARN("update state with new odom on IMU frame is fail");
            if(!ncrl_tf::accumTrans(frame_j->second.Tw2curr_W, WORLD2IMU, frame_j->second.Tw2curr_I))
                ROS_WARN("update state with new odom on WORLD frame is fail");
        }
        count++;
    }
}

void ncrl_lio_feature_manager::UpdateAndFindMapFeaturePoint(FeaturePerSweep &Sweep_c){
    int centerCubeI = int((Sweep_c.Tw2curr_W.v.x() + cubeSizeHalf) / cubeSize) + Sweep_c.laserCloudCenWidth;
    int centerCubeJ = int((Sweep_c.Tw2curr_W.v.y() + cubeSizeHalf) / cubeSize) + Sweep_c.laserCloudCenHeight;
    int centerCubeK = int((Sweep_c.Tw2curr_W.v.z() + cubeSizeHalf) / cubeSize) + Sweep_c.laserCloudCenDepth;

    if (Sweep_c.Tw2curr_W.v.x() + cubeSizeHalf < 0)
        centerCubeI--;
    if (Sweep_c.Tw2curr_W.v.y() + cubeSizeHalf < 0)
        centerCubeJ--;
    if (Sweep_c.Tw2curr_W.v.z() + cubeSizeHalf < 0)
        centerCubeK--;

    while (centerCubeI < 3)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i >= 1; i--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI++;
        Sweep_c.laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i < laserCloudWidth - 1; i++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        Sweep_c.laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = laserCloudHeight - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j >= 1; j--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        Sweep_c.laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j < laserCloudHeight - 1; j++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        Sweep_c.laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = laserCloudDepth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k >= 1; k--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        Sweep_c.laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k < laserCloudDepth - 1; k++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK--;
        Sweep_c.laserCloudCenDepth--;
    }

    Sweep_c.laserCloudValidNum = 0;
    Sweep_c.laserCloudSurroundNum = 0;

    // update laserCloudValidNum
    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
            for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
            {
                if (i >= 0 && i < laserCloudWidth &&
                    j >= 0 && j < laserCloudHeight &&
                    k >= 0 && k < laserCloudDepth)
                {
                    Sweep_c.laserCloudValidInd[Sweep_c.laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    Sweep_c.laserCloudValidNum++;
                    Sweep_c.laserCloudSurroundInd[Sweep_c.laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    Sweep_c.laserCloudSurroundNum++;
                }
            }
        }
    }

    // put the map's feature point into [feature]FromMap
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();

    for (int i = 0; i < Sweep_c.laserCloudValidNum; i++)
    {
        *laserCloudCornerFromMap += *laserCloudCornerArray[Sweep_c.laserCloudValidInd[i]];
        *laserCloudSurfFromMap += *laserCloudSurfArray[Sweep_c.laserCloudValidInd[i]];
    }

    laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
    laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

//    // downsize last feature point into stack and update num
//    Sweep_c.laserCloudCornerStack->clear();
//    Sweep_c.laserCloudSurfStack->clear();
//    downSizeFilter(Sweep_c.laserCloudCornerLast, Sweep_c.laserCloudCornerStack, lineRes);
//    downSizeFilter(Sweep_c.laserCloudSurfLast, Sweep_c.laserCloudSurfStack, planeRes);
//    Sweep_c.laserCloudCornerStackNum = Sweep_c.laserCloudCornerStack->points.size();
//    Sweep_c.laserCloudSurfStackNum = Sweep_c.laserCloudSurfStack->points.size();
}

void ncrl_lio_feature_manager::SetInputMapKdTree(pcl::PointCloud<PointType>::Ptr cornerMap, pcl::PointCloud<PointType>::Ptr surfaceMap){
    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
}

void ncrl_lio_feature_manager::pointAssociateToMap(PointType const *const pi, PointType *const po,
                                                   Eigen::Quaterniond q_w_curr, Eigen::Vector3d t_w_curr){
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

bool ncrl_lio_feature_manager::KSearchMapKdTree(PointType pSel, int num, std::vector<int> &Ind, std::vector<float> &Dis, int flag){
    if (flag == 0){
        kdtreeCornerFromMap->nearestKSearch(pSel, num, Ind, Dis);
    } else if (flag == 1){
        kdtreeSurfFromMap->nearestKSearch(pSel, num, Ind, Dis);
    } else {
        ROS_WARN("wrong flag");
        return false;
    }
    return true;
}

// need each frame's rotation and pre_integration
void ncrl_lio_feature_manager::solveGyroscopeBias(Eigen::Vector3d* Bgs){
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    Eigen::Vector3d delta_bg;
    A.setZero(3, 3);
    b.setZero(3, 1);
    std::map<int, FeaturePerSweep>::iterator frame_i;
    std::map<int, FeaturePerSweep>::iterator frame_j;
    for (frame_i = Feature_All_Map.begin(); std::next(frame_i) != Feature_All_Map.end(); frame_i++){
        frame_j = std::next(frame_i);
        Eigen::Matrix3d tmp_A;
        tmp_A.setZero(3, 3);
        Eigen::Vector3d tmp_b;
        tmp_b.setZero(3, 1);

        Eigen::Quaterniond q_ij = frame_i->second.Tinit2odom_I.q.inverse() * frame_j->second.Tinit2odom_I.q;
        // Jacobian is 15 * 15 matrix (O_R, O_BG) = (3, 12)
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());
    for (int i = 0; i < SWEEP_SIZE + 1; i++)
        Bgs[i] += delta_bg;
    for (frame_i = Feature_All_Map.begin(); std::next(frame_i) != Feature_All_Map.end(); frame_i ++){
        frame_j = std::next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

bool ncrl_lio_feature_manager::LinearAlignment(Eigen::Vector3d &g){
    int all_sweep_count = Feature_All_Map.size();
    int n_state = all_sweep_count * 3 + 3; // all_sweep_count * 3 velocity, 3 : gravity, 1 : scale
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero(n_state, n_state);
    VectorXd b{n_state};
    b.setZero(n_state, 1);

    std::map<int, FeaturePerSweep>::iterator frame_i;
    std::map<int, FeaturePerSweep>::iterator frame_j;
    int i = 0;
    // now window is 2
    for (frame_i = Feature_All_Map.begin(); std::next(frame_i) != Feature_All_Map.end(); frame_i++, i++){
        frame_j = std::next(frame_i);
        Eigen::MatrixXd tmp_A(6, 9);
        tmp_A.setZero(6, 9);
        Eigen::VectorXd tmp_b(6);
        tmp_b.setZero(6, 1);

        double dt = frame_j->second.pre_integration->sum_dt;
        // imu odom to imu frame
        Eigen::Matrix3d frame_i_w_b = frame_i->second.Tinit2odom_I.q.normalized().toRotationMatrix();
        Eigen::Matrix3d frame_j_w_b = frame_j->second.Tinit2odom_I.q.normalized().toRotationMatrix();

        tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = 0.5 * frame_i_w_b.transpose() * dt * dt * Eigen::Matrix3d::Identity();
//        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_i_w_b.transpose() * (frame_j->second.t_w_curr - frame_i->second.t_w_curr);
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_i_w_b.transpose() * (frame_j->second.Tinit2odom_I.v - frame_i->second.Tinit2odom_I.v);

        tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i_w_b.transpose() * frame_j_w_b;
        tmp_A.block<3, 3>(3, 6) = frame_i_w_b.transpose() * dt * Eigen::Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v : " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>(); // 30
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>(); // 4
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    Eigen::VectorXd x = A.ldlt().solve(b); // 34 * 1
    g = x.segment<3>(n_state - 3);
    ROS_DEBUG_STREAM(" before g     " << g.norm() << " " << g.transpose());

    // refine gravity
    if(fabs(g.norm() - G.norm()) > 1.0) {
        return false;
    }
    else {
        g = g * G.norm() / g.norm();
        return true;
    }
}

bool ncrl_lio_feature_manager::SolveGravity(Eigen::Vector3d &g){
    Eigen::MatrixXd A{6, 3};
    A.setZero(6, 3);
    VectorXd b{6};
    b.setZero(6, 1);

    std::map<int, FeaturePerSweep>::iterator frame_i;
    std::map<int, FeaturePerSweep>::iterator frame_j;
    int i = 0;
    for (frame_i = Feature_All_Map.begin(); std::next(frame_i) != Feature_All_Map.end(); frame_i++, i++){
        frame_j = std::next(frame_i);
        Eigen::MatrixXd tmp_A(6, 3);
        tmp_A.setZero(6, 3);
        Eigen::VectorXd tmp_b(6);
        tmp_b.setZero(6, 1);

        double dt = frame_j->second.pre_integration->sum_dt;
        Eigen::Matrix3d frame_i_w_b = frame_i->second.Tinit2odom_I.q.normalized().toRotationMatrix();
        Eigen::Matrix3d frame_j_w_b = frame_j->second.Tinit2odom_I.q.normalized().toRotationMatrix();
        Eigen::Vector3d Pj = frame_j->second.Tinit2odom_I.v;
        Eigen::Vector3d Pi = frame_i->second.Tinit2odom_I.v;
        Eigen::Vector3d Vij = frame_i_w_b.transpose() * (Pj - Pi)/(frame_j->second.laserOdom.header.stamp.toSec() -
                                         frame_i->second.laserOdom.header.stamp.toSec());
        ROS_DEBUG_STREAM("Vij:" << Vij.transpose());
        tmp_A.block<3, 3>(0, 0) = 0.5 * frame_i_w_b.transpose() * dt * dt;
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_i_w_b.transpose() * (Pj - Pi)
                                  + dt * Vij;

        tmp_A.block<3, 3>(3, 0) = frame_i_w_b.transpose() * dt;
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v
                                  + (Eigen::Matrix3d::Identity() - frame_i_w_b.transpose() * frame_j_w_b) * Vij;

        A += tmp_A;
        b += tmp_b;
    }
    A = A * 1000.0;
    b = b * 1000.0;
    g = A.colPivHouseholderQr().solve(b); // 34 * 1
    ROS_DEBUG_STREAM(" before g " << g.norm() << " " << g.transpose());

    // refine gravity
    if (fabs(g.norm() - G.norm()) > 1.0) {
        return false;
    }
    else {
        g = g * G.norm() / g.norm();
        return true;
    }
}

// maintain the size after intialStructure success
void ncrl_lio_feature_manager::initSizeMaintainer(){
    while ((int)Feature_All_Map.size() > SWEEP_SIZE + 1) {
        Feature_All_Map.erase(Feature_All_Map.begin());
    }

    // initial state : from WORLD to IMU_INIT
    map<int, FeaturePerSweep>::reverse_iterator frame_i = Feature_All_Map.rbegin();
    initGuess_I.q = frame_i->second.Tinit2odom_I.q;
    initGuess_I.v = frame_i->second.Tinit2odom_I.v;
    initGuess_L.q = frame_i->second.Tinit2odom_L.q;
    initGuess_L.v = frame_i->second.Tinit2odom_L.v;
}


// pointSel should on map/world frame
bool ncrl_lio_feature_manager::LidarPlaneNormHandler(PointType pointSel, Eigen::Vector3d &norm, double &negative_OA_dot_norm){
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  if(KSearchMapKdTree(pointSel, 5, pointSearchInd, pointSearchSqDis, SURFACE)){
    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0){
      for (int j = 0; j < 5; j++){
        matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
        matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
        matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
      }
      norm = matA0.colPivHouseholderQr().solve(matB0);
      negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();
      for (int j = 0; j < 5; j++)
      {
          // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
        {
            return false;
        }
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

// pointSel should on map/world frame
bool ncrl_lio_feature_manager::LidarEdgeHandler(PointType pointSel, Eigen::Vector3d &point_a, Eigen::Vector3d &point_b){
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    if (KSearchMapKdTree(pointSel, 5, pointSearchInd, pointSearchSqDis, CORNER)){
        if (pointSearchSqDis[4] < 1.0){
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++){
                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]){
                Eigen::Vector3d point_on_line = center;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

bool ncrl_lio_feature_manager::LidarF2fPlaneHandler(pcl::PointCloud<PointType>::Ptr laserCloudSurfLast, PointType pointSel,
                                                    int &closestPointInd, int &minPointInd2, int &minPointInd3){
    // get closest point's scan ID
    int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
    double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
    // search in the direction of increasing scan line
    for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j){
        // if not in nearby scans, end the loop
        if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
            break;

        double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                (laserCloudSurfLast->points[j].z - pointSel.z);

        // if in the same or lower scan line
        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2){
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
        }
        // if in the higher scan line
        else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3){
            minPointSqDis3 = pointSqDis;
            minPointInd3 = j;
        }
    }
    // search in the direction of decreasing scan line
    for (int j = closestPointInd - 1; j >= 0; --j){
        // if not in nearby scans, end the loop
        if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
            break;

        double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                (laserCloudSurfLast->points[j].z - pointSel.z);

        // if in the same or higher scan line
        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
        }
        else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
            // find nearer point
            minPointSqDis3 = pointSqDis;
            minPointInd3 = j;
        }
    }
    if (minPointInd2 >= 0 && minPointInd3 >= 0)
        return true;
    else
        return false;
}

bool ncrl_lio_feature_manager::LidarF2fEdgeHandler(pcl::PointCloud<PointType>::Ptr laserCloudCornerLast, PointType pointSel,
                                                   int &closestPointInd, int &minPointInd2){
    int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity + 1000);

    double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
    // search in the direction of increasing scan line
    for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
    {
        // if in the same scan line, continue
        if (int(laserCloudCornerLast->points[j].intensity + 1000) <= closestPointScanID)
            continue;

        // if not in nearby scans, end the loop
        if (int(laserCloudCornerLast->points[j].intensity + 1000) > (closestPointScanID + NEARBY_SCAN))
            break;

        double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                (laserCloudCornerLast->points[j].x - pointSel.x) +
                            (laserCloudCornerLast->points[j].y - pointSel.y) *
                                (laserCloudCornerLast->points[j].y - pointSel.y) +
                            (laserCloudCornerLast->points[j].z - pointSel.z) *
                                (laserCloudCornerLast->points[j].z - pointSel.z);

        if (pointSqDis < minPointSqDis2)
        {
            // find nearer point
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
        }
    }

    // search in the direction of decreasing scan line
    for (int j = closestPointInd - 1; j >= 0; --j)
    {
        // if in the same scan line, continue
        if (int(laserCloudCornerLast->points[j].intensity + 1000) >= closestPointScanID)
            continue;

        // if not in nearby scans, end the loop
        if (int(laserCloudCornerLast->points[j].intensity + 1000) < (closestPointScanID - NEARBY_SCAN))
            break;

        double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                (laserCloudCornerLast->points[j].x - pointSel.x) +
                            (laserCloudCornerLast->points[j].y - pointSel.y) *
                                (laserCloudCornerLast->points[j].y - pointSel.y) +
                            (laserCloudCornerLast->points[j].z - pointSel.z) *
                                (laserCloudCornerLast->points[j].z - pointSel.z);

        if (pointSqDis < minPointSqDis2)
        {
            // find nearer point
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
        }
    }
    if (minPointInd2 >= 0)
        return true;
    else
        return false;

}

void ncrl_lio_feature_manager::RANSAC_segmentation(pcl::PointCloud<PointType>::Ptr &foreground,
                                                   pcl::PointCloud<PointType>::Ptr &background)
{
    for (int i = 0; i < 3; i++) {
       // Create the segmentation object for the planar model and set all the parameters
       pcl::SACSegmentation<pcl::PointXYZI> seg;
       pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
       pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
       seg.setOptimizeCoefficients (true);
       seg.setModelType (pcl::SACMODEL_PLANE);
       seg.setMethodType (pcl::SAC_RANSAC);
       seg.setMaxIterations (100);
       seg.setDistanceThreshold (BackgroundDis);
       seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
       seg.setEpsAngle(30.0f * (M_PI/180.0f));

       // Segment the largest planar component from the remaining cloud
       seg.setInputCloud (background);
       seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0) {
           std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
       }

       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZI> extract;
       extract.setInputCloud (background);
       extract.setIndices (inliers);
       extract.setNegative (false);

       // Get the inliers points associated with the planar surface
       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI>());
       extract.filter (*cloud_plane);
       // Extract the rest of the points
       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
       extract.setNegative (true);
       extract.filter (*cloud_filter);
       *foreground += *cloud_filter;
       *background = *cloud_plane;

//       ROS_WARN("seg %d, %d", cloud_plane->points.size(), cloud_filter->points.size());
       if (cloud_filter->points.size() < 100)
         break;
    }
}

void ncrl_lio_feature_manager::background_extraction(pcl::PointCloud<PointType>::Ptr input_cloud,
                                                     pcl::PointCloud<PointType>::Ptr &foreground,
                                                     pcl::PointCloud<PointType>::Ptr &background)
{
    double vertical_sum = 0;
    for (int i = 0; i < (int)input_cloud->points.size(); i++) {
        vertical_sum += input_cloud->points[i].z;
    }
    double vertical_mean = vertical_sum/input_cloud->points.size();
//    double diff_square = 0;
//    for (int i = 0; i < (int)input_cloud->points.size(); i++) {
//        diff_square += (input_cloud->points[i].z - vertical_mean) * (input_cloud->points[i].z - vertical_mean);
//    }
//    double z_var = sqrt(diff_square/input_cloud->points.size());

    for (int i = 0; i < (int)input_cloud->points.size(); i++) {
        PointType pointSel = input_cloud->points[i];
        if ((pointSel.z - vertical_mean) < 0) {
            background->points.push_back(pointSel);
        }
        else
          foreground->points.push_back(pointSel);
    }
    RANSAC_segmentation(foreground, background);
}

void ncrl_lio_feature_manager::pointL2I(Eigen::Vector3d p_i, PointType& p_o){
    ncrl_tf::Point p_;
    ncrl_tf::setPoint(p_, p_i);
    ncrl_tf::setPointFrame(p_, "LIDAR");
    if(!ncrl_tf::TransPoint(EXTRINSIC, p_))
        ROS_WARN("trans plane point to imu frame");
    p_o.x = p_.point.x();
    p_o.y = p_.point.y();
    p_o.z = p_.point.z();
}

void ncrl_lio_feature_manager::UpdateMap(){
    map<int, FeaturePerSweep>::reverse_iterator frame_i = Feature_All_Map.rbegin();
    if (1){
        laserCloudStatic->clear();
        pcl::PointCloud<PointType>::Ptr feature_cloud(new pcl::PointCloud<PointType>());
        *feature_cloud += *frame_i->second.laserCloudCornerStack;
        *feature_cloud += *frame_i->second.laserCloudSurfStack;

        pcl::PointCloud<PointType>::Ptr feature_cloud_(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)feature_cloud->points.size(); i++) {
            PointType pointOri = feature_cloud->points[i];
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

            // tranfrom from lidar to imu
            pointL2I(curr_point, pointOri);

            feature_cloud_->points.push_back(pointOri);
        }

        pcl::PointCloud<PointType>::Ptr foreground(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr background(new pcl::PointCloud<PointType>());
        background_extraction(feature_cloud_, foreground, background);

        // construct map from foreground
        for (int i = 0; i < (int)foreground->points.size(); i++){
            PointType pointSel = foreground->points[i];
            // trans point to WORLD frame
            pointAssociateToMap(&pointSel, &pointSel,
                                             frame_i->second.Tw2curr_W.q, frame_i->second.Tw2curr_W.v);
            int cubeI = int((pointSel.x + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenWidth;
            int cubeJ = int((pointSel.y + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenHeight;
            int cubeK = int((pointSel.z + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenDepth;

            if (pointSel.x + cubeSizeHalf < 0)
                cubeI--;
            if (pointSel.y + cubeSizeHalf < 0)
                cubeJ--;
            if (pointSel.z + cubeSizeHalf < 0)
                cubeK--;
            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudStatic->push_back(pointSel);
                if (pointSel.intensity < -949 && pointSel.intensity > -1001){
                    laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
                else if (pointSel.intensity < 51 && pointSel.intensity > -1){
                    laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
                else{
                    ROS_WARN("wrong feature type!!!");
                }
            }
        }

        // construct map from background
        for (int i = 0; i < (int)background->points.size(); i++){
            PointType pointSel = background->points[i];
            // trans point to WORLD frame
            pointAssociateToMap(&pointSel, &pointSel,
                                             frame_i->second.Tw2curr_W.q, frame_i->second.Tw2curr_W.v);
            int cubeI = int((pointSel.x + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenWidth;
            int cubeJ = int((pointSel.y + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenHeight;
            int cubeK = int((pointSel.z + cubeSizeHalf) / cubeSize) + frame_i->second.laserCloudCenDepth;
            if (pointSel.x + cubeSizeHalf < 0)
                cubeI--;
            if (pointSel.y + cubeSizeHalf < 0)
                cubeJ--;
            if (pointSel.z + cubeSizeHalf < 0)
                cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
            {
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                if (ENABLE_REMOVE_BACKGROUND) {
                    //point will not store in array
                } else {
                    laserCloudStatic->push_back(pointSel);
                }
                if (pointSel.intensity < -949 && pointSel.intensity > -1001){
                    laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
                else if (pointSel.intensity < 51 && pointSel.intensity > -1){
                    laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
                else{
                    ROS_WARN("wrong feature type!!!");
                }
            }
        }

        // using voxel filter to maintain intensity of map points
        for (int i = 0; i < frame_i->second.laserCloudValidNum; i++){
            int ind = frame_i->second.laserCloudValidInd[i];
            pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
            downSizeFilter(laserCloudCornerArray[ind],
                                        tmpCorner, lineRes);
            downSizeFilter(laserCloudSurfArray[ind],
                                        tmpSurf, planeRes);
            laserCloudCornerArray[ind] = tmpCorner;
            laserCloudSurfArray[ind] = tmpSurf;
        }
    }

    // Generate visualization msgs : scan
    int laserCloudFullResNum = frame_i->second.laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++){
        PointType pointOri = frame_i->second.laserCloudFullRes->points[i];
        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        pointL2I(curr_point, pointOri);
        pointAssociateToMap(&pointOri,
                             &frame_i->second.laserCloudFullRes->points[i],
                              frame_i->second.Tw2curr_W.q, frame_i->second.Tw2curr_W.v);
    }
//    laserCloudRegisted->clear();
    laserCloudRegisted = frame_i->second.laserCloudFullRes;
}
