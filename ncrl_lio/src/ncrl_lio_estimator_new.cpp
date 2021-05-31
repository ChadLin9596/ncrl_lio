#include "ncrl_lio/ncrl_lio_estimator_new.h"

ncrl_lio_estimator_new::ncrl_lio_estimator_new(){
    for (int i = 0; i < SWEEP_SIZE + 1; i++)
    {
        Qs[i].setIdentity();
        Ps[i].setZero(3,1);
        Vs[i].setZero(3,1);
        Bas[i].setZero(3,1);
        Bgs[i].setZero(3,1);
    }
    til[0] = Eigen::Vector3d::Zero();
    qil[0] = Eigen::Quaterniond::Identity();
    w2init[0] = Eigen::Quaterniond::Identity();
    acc_0 = Eigen::Vector3d::Zero();
    gyr_0 = Eigen::Vector3d::Zero();
    ncrl_tf::setPointFrame(g, "WORLD");
    ncrl_tf::setPoint(g, G);

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    tmp_pre_integration = nullptr;

    // marginalization setting
    marginalization_flag = MARGIN_OLD;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    Feature_All = new ncrl_lio_feature_manager;
}

void ncrl_lio_estimator_new::processImu(double dt, const Vector3d &linear_acceleration,
                                                   const Vector3d &angular_velocity){
    if (!first_imu){
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // initial
    if(!tmp_pre_integration)
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[SWEEP_SIZE], Bgs[SWEEP_SIZE]};

    // ignore the first combined data, since the datas aren't sufficient in 0.1 sec
    if (frame_count != 0) // frame_count 1 2 3 ... 9 10
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/*
 * general : margin old
 */
void ncrl_lio_estimator_new::maintainSize(){
    if (marginalization_flag == MARGIN_OLD) {
        if (frame_count == SWEEP_SIZE){
            if (solver_flag == NON_LINEAR){
                // target : maintain Feature_All_Map size is SWEEP_SIZE
                while ((int)Feature_All->Feature_All_Map.size() > SWEEP_SIZE)
                    Feature_All->Feature_All_Map.erase(Feature_All->Feature_All_Map.begin());
            }

            // only sweep_size + 1 > 1 need maintain state size
            // both solver_flag = INITIAL or NONLINEAR will
            if (SWEEP_SIZE >= 1){
                for (int i = 0; i < SWEEP_SIZE; i++){
                    Ps[i].swap(Ps[i+1]);
                    std::swap(Qs[i], Qs[i+1]);
                    Vs[i].swap(Vs[i+1]);
                    Bas[i].swap(Bas[i+1]);
                    Bgs[i].swap(Bgs[i+1]);
                }
                Ps[SWEEP_SIZE] = Ps[SWEEP_SIZE - 1];
                Qs[SWEEP_SIZE] = Qs[SWEEP_SIZE - 1];
                Vs[SWEEP_SIZE] = Vs[SWEEP_SIZE - 1];
                Bas[SWEEP_SIZE] = Bas[SWEEP_SIZE - 1];
                Bgs[SWEEP_SIZE] = Bgs[SWEEP_SIZE - 1];
            }
        }
    }
}

void ncrl_lio_estimator_new::processLIO(FeaturePerSweep &feature_msg, int lidar_id){
    feature_msg.id = lidar_id;
    feature_msg.pre_integration = tmp_pre_integration;
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[SWEEP_SIZE], Bgs[SWEEP_SIZE]};

    // insert into Container which size should be SWEEP_SIZE + 1
    Feature_All->Feature_All_Map.insert(std::make_pair(lidar_id, feature_msg));

    // TARGET of INITIAL : initial extrinsic value, imu bias, rotation of world to imu
    if (solver_flag == INITIAL) {
        if (initialStructure()){
            // restore the frame_count for optimization windows
            frame_count = SWEEP_SIZE;

            // add initialGuess
            Feature_All->UpdateState(solver_flag);
            solver_flag = NON_LINEAR;
        } else {
            ROS_INFO_STREAM("ACCUMULATE THE COMBINED DATE WITH " << frame_count);
            frame_count++;
        }
    } else
        Feature_All->UpdateState(solver_flag);

    if (solver_flag == NON_LINEAR){
        optimization();
        maintainSize();
    }
}

bool ncrl_lio_estimator_new::checkIMUexcitation(){
    Vector3d sum_g(0,0,0);
    std::map<int, FeaturePerSweep>::iterator sweep_it;
    // do 2 - 10 data(total 1 - 10)
    for (sweep_it = Feature_All->Feature_All_Map.begin(), sweep_it++; sweep_it!= Feature_All->Feature_All_Map.end(); sweep_it++){
        double dt = sweep_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = sweep_it->second.pre_integration->delta_v/dt;
        sum_g += tmp_g;
    }

    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)Feature_All->Feature_All_Map.size() - 1);
    double var = 0;

    for (sweep_it = Feature_All->Feature_All_Map.begin(), sweep_it++; sweep_it!= Feature_All->Feature_All_Map.end(); sweep_it++){
        double dt = sweep_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = sweep_it->second.pre_integration->delta_v/dt;
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }

    var = sqrt(var / ((int)Feature_All->Feature_All_Map.size() - 1));
    if(var < 0.25){
        ROS_INFO("IMU excitation not enough");
        return false;
    } else{
        ROS_DEBUG("IMU excitation enough");
        return true;
    }
}

void ncrl_lio_estimator_new::showTimeCost(){
    cout << "===== COST OF LIO ESTIMATOR =====" << endl;
    t_count++;
    t_whole_sum += t_whole;
    if (t_whole_max < t_whole)
        t_whole_max = t_whole;
    if (t_cereSolver >= 0)
        ROS_DEBUG_STREAM("cere solver  cost  : " << t_cereSolver << " ms");
    if (t_whole >= 0)
        ROS_DEBUG_STREAM("whole system cost : " << t_whole << " ms");
    if (t_whole_max >= 0)
        ROS_DEBUG_STREAM("whole mean   cost : " << t_whole_sum / t_count << " ms");
    if (t_whole_max >= 0)
        ROS_DEBUG_STREAM("whole max    cost : " << t_whole_max << " ms");
    t_whole = -1;
    t_cereSolver = -1;
    cout << "===== COST OF LIO ESTIMATOR =====" << endl;
}

bool ncrl_lio_estimator_new::initialStructure(){
    // check imu excitation
#if 0
    if (!checkIMUexcitation())
        return false;
#endif
    if (frame_count >= INIT_SIZE){
        int size = Feature_All->Feature_All_Map.size();
        if (size != INIT_SIZE + 1)
            ROS_WARN_STREAM("SIZE IN INITIALSTRUCTURE IS " << size << "! IT SHOULD BE " << INIT_SIZE + 1);

        return LidarImuAlign();
    } else {
        ROS_INFO_STREAM("ACCUMULATE THE COMBINED DATE WITH " << frame_count);
        return false;
    }
}

bool ncrl_lio_estimator_new::LidarImuAlign(){
    // do on ncrl_lio_feature_manager
    Feature_All->solveGyroscopeBias(Bgs);

    bool result = false;
    if (ENABLE_HIGHSPEED_INITIALIZATION)
      result = Feature_All->SolveGravity(g.point);
    else
      result = Feature_All->LinearAlignment(g.point);

    ROS_DEBUG_STREAM(" result g " << g.point.norm() << " " << g.point.transpose());
    if (!result)
      return result;

    // calibrate the rotation from world to imu and set the rotation as initial state
    g.frame = "IMU_INIT";
    Eigen::Quaterniond q_ = Eigen::Quaterniond::FromTwoVectors(g.point, G);
    Eigen::Vector3d euler = Q2RPY(q_);

    // update state from imu init frame to world frame
    WORLD2IMU.v = q_ * WORLD2IMU.v;
    WORLD2IMU.q = q_ * WORLD2IMU.q;
    ROS_DEBUG_STREAM("EULER CALIBRATION FROM WORLD TO IMU_INIT : " << rad2deg(euler).transpose() << " deg");
    Feature_All->initSizeMaintainer();
    return result;
}

void ncrl_lio_estimator_new::vector2double(){
    map<int, FeaturePerSweep>::iterator frame_i;
    int cnt_ = 0;
    for (frame_i = Feature_All->Feature_All_Map.begin(); frame_i != Feature_All->Feature_All_Map.end(); frame_i++) {
        Ps[cnt_] = frame_i->second.Tw2curr_W.v;
        Qs[cnt_] = frame_i->second.Tw2curr_W.q;
        cnt_++;
    }

    for (int i = 0; i < SWEEP_SIZE + 1; i++) {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        para_Pose[i][3] = Qs[i].x();
        para_Pose[i][4] = Qs[i].y();
        para_Pose[i][5] = Qs[i].z();
        para_Pose[i][6] = Qs[i].w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    // setting extrinsic state
    para_Ex_Pose[0][0] = til[0].x();
    para_Ex_Pose[0][1] = til[0].y();
    para_Ex_Pose[0][2] = til[0].z();
    para_Ex_Pose[0][3] = qil[0].x();
    para_Ex_Pose[0][4] = qil[0].y();
    para_Ex_Pose[0][5] = qil[0].z();
    para_Ex_Pose[0][6] = qil[0].w();
}

void ncrl_lio_estimator_new::double2vector(){
    for (int i = 0; i < SWEEP_SIZE + 1; i++){
        Qs[i] = Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]);
        Ps[i] = Eigen::Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        Vs[i] = Vector3d(para_SpeedBias[i][0],
                         para_SpeedBias[i][1],
                         para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    // setting extrinsic state
    Eigen::Vector3d v_(para_Ex_Pose[0][0],
                       para_Ex_Pose[0][1],
                       para_Ex_Pose[0][2]);
    Eigen::Quaterniond q_(para_Ex_Pose[0][6],
                          para_Ex_Pose[0][3],
                          para_Ex_Pose[0][4],
                          para_Ex_Pose[0][5]);
    til[0] = v_;
    qil[0] = q_;
    if (ENABLE_OPT_EXTRINSIC)
        ncrl_tf::setTrans(EXTRINSIC, q_, v_);
}

void ncrl_lio_estimator_new::optimization(){
    // update and construct map
    Feature_All->UpdateAndMaintainMapCube_All(solver_flag);

    vector2double();

    // declare cere-solver and add state into problem
//    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.5);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // state update setting
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization;
    vector<double *> para_ids;

    // setting state and local parameterization
    for (int i = 0; i < SWEEP_SIZE + 1; i++){
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        para_ids.push_back(para_Pose[i]);
        para_ids.push_back(para_SpeedBias[i]);
    }

    problem.AddParameterBlock(para_Ex_Pose[0] , SIZE_POSE, local_parameterization);
    if (!ENABLE_OPT_EXTRINSIC)
        problem.SetParameterBlockConstant(para_Ex_Pose[0]);

    vector<ceres::internal::ResidualBlock *> res_ids_imu;
    vector<ceres::internal::ResidualBlock *> res_ids_edge;
    vector<ceres::internal::ResidualBlock *> res_ids_plane;
    vector<ceres::internal::ResidualBlock *> res_ids_marg;
    vector<ceres::internal::ResidualBlock *> res_ids_f2f_plane;
    vector<ceres::internal::ResidualBlock *> res_ids_f2f_edge;

    // Marginalization Cost
    if (ENABLE_MARGINALIZATION && last_marginalization_info) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ceres::internal::ResidualBlock *res_id_marg =
            problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
        res_ids_marg.push_back(res_id_marg);
    }

    // construct imu cost function
    map<int, FeaturePerSweep>::iterator sweep_i;
    int i = 0;
    for (sweep_i = Feature_All->Feature_All_Map.begin(); next(sweep_i)!= Feature_All->Feature_All_Map.end(); sweep_i++, i++){
        map<int, FeaturePerSweep>::iterator sweep_j = std::next(sweep_i);
        if (sweep_j->second.pre_integration->sum_dt > 10.0)
            continue;

        IMUFactor* imu_factor = new IMUFactor(sweep_j->second.pre_integration);
        ceres::internal::ResidualBlock *res_id =
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                                                       para_Pose[i + 1], para_SpeedBias[i + 1]);
        res_ids_imu.push_back(res_id);
    }

    if (ENABLE_PRIOR_FACTOR){
        ExtrinFactor* extrin_factor = new ExtrinFactor(til[0], qil[0], w_ext_tran, w_ext_rot);
        problem.AddResidualBlock(extrin_factor, NULL, para_Ex_Pose[0]);
    }

    // declare the variable of f2f
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());

    // construct kd tree with feature point on map
    if (Feature_All->laserCloudCornerFromMapNum > 0 && Feature_All->laserCloudSurfFromMapNum > 0)
        Feature_All->SetInputMapKdTree(Feature_All->laserCloudCornerFromMap, Feature_All->laserCloudSurfFromMap);

    // construct Lidar cost function, total have # of feature points * Sweep_size
    bool enoughSolved = true;
    int frameCount = 0;
    map<int, FeaturePerSweep>::iterator frame_i;
    for (frame_i = Feature_All->Feature_All_Map.begin(); frame_i != Feature_All->Feature_All_Map.end(); frame_i++) {
        if (Feature_All->laserCloudCornerFromMapNum > 10 && Feature_All->laserCloudSurfFromMapNum > 50) {
            // construct corner cost function
            if (ENABLE_F2M_EDGE) {
                for (int i = 0; i < frame_i->second.laserCloudCornerStackNum; i++) {
                    PointType pointOri;
                    PointType pointSel;

                    pointOri = frame_i->second.laserCloudCornerStack->points[i];
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

                    // tranfrom from lidar to imu
                    Feature_All->pointL2I(curr_point, pointOri);

                    // use initial frame's result to find the closest point in kd tree
                    Feature_All->pointAssociateToMap(&pointOri, &pointSel, Qs[frameCount], Ps[frameCount]);

                    Eigen::Vector3d point_a, point_b;
                    if (Feature_All->LidarEdgeHandler(pointSel, point_a, point_b)) {
                        LidarEdgeFactor *cost_function = new LidarEdgeFactor(curr_point, point_a, point_b, 1.0, w_f2m_corner);
                        ceres::internal::ResidualBlock *res_id =
                            problem.AddResidualBlock(cost_function, loss_function, para_Pose[frameCount], para_Ex_Pose[0]);
                        res_ids_edge.push_back(res_id);
                    }
                } //for (int i = 0; i < Feature_All->laserCloudCornerStackNum; i++)
            }

            // construct surface cost function
            if (ENABLE_F2M_PLANE) {
                for (int i = 0; i < frame_i->second.laserCloudSurfStackNum; i++){
                    PointType pointOri;
                    PointType pointSel;
                    pointOri = frame_i->second.laserCloudSurfStack->points[i];
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    // tranfrom from lidar to imu
                    Feature_All->pointL2I(curr_point, pointOri);

                    // trans from imu to world frame
                    Feature_All->pointAssociateToMap(&pointOri, &pointSel, Qs[frameCount], Ps[frameCount]);

                    Eigen::Vector3d norm;
                    double negative_OA_dot_norm;
                    // preprocess of constructing plane cost function
                    if (Feature_All->LidarPlaneNormHandler(pointSel, norm, negative_OA_dot_norm)){
                        LidarPlaneNormFactor *cost_function = new LidarPlaneNormFactor(curr_point, norm, negative_OA_dot_norm, w_f2m_flat);
                        ceres::internal::ResidualBlock *res_id =
                            problem.AddResidualBlock(cost_function, loss_function, para_Pose[frameCount], para_Ex_Pose[0]);
                        res_ids_plane.push_back(res_id);
                    }
                } // for (int i = 0; i < Feature_All->laserCloudSurfStackNum; i++)
            }

            // add f2f residual
            if (ENABLE_F2F_PLANE){
                if(frame_i->second.laserCloudSurfStackNum > 0 && frameCount != SWEEP_SIZE){
                    auto frame_j = std::next(frame_i);
                    laserCloudSurfLast = frame_i->second.laserCloudSurfStack;
                    Feature_All->kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

                    // trans j lidar frame point into i lidar frame
                    ncrl_tf::Trans Tw2lj, Tw2li, Tli2lj, tmpExtrinisc;
                    ncrl_tf::setTransFrame(Tw2lj, "WORLD", "LIDAR_J");
                    ncrl_tf::setTransFrame(Tw2li, "WORLD", "LIDAR_I");
                    ncrl_tf::setTransFrame(Tli2lj, "LIDAR_I", "LIDAR_J");

                    tmpExtrinisc = EXTRINSIC;
                    ncrl_tf::setTransFrame(tmpExtrinisc, "IMU", "LIDAR_J");
                    if (!ncrl_tf::accumTrans(Tw2lj, frame_j->second.Tw2curr_W, tmpExtrinisc))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    ncrl_tf::setTransFrame(tmpExtrinisc, "IMU", "LIDAR_I");
                    if (!ncrl_tf::accumTrans(Tw2li, frame_i->second.Tw2curr_W, tmpExtrinisc))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    if (!ncrl_tf::deltaTrans(Tli2lj, Tw2li, Tw2lj))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    for (int i = 0; i < (int)frame_j->second.laserCloudSurfStackNum; i++) {
                        PointType pointOri;
                        PointType pointSel;
                        std::vector<int> pointSearchInd;
                        std::vector<float> pointSearchSqDis;
                        pointOri = frame_j->second.laserCloudSurfStack->points[i];

                        Feature_All->pointAssociateToMap(&pointOri, &pointSel, Tli2lj.q, Tli2lj.v);
                        Feature_All->kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                            closestPointInd = pointSearchInd[0];

                            if (Feature_All->LidarF2fPlaneHandler(laserCloudSurfLast, pointSel, closestPointInd, minPointInd2, minPointInd3)) {
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                             laserCloudSurfLast->points[closestPointInd].y,
                                                             laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                             laserCloudSurfLast->points[minPointInd2].y,
                                                             laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                             laserCloudSurfLast->points[minPointInd3].y,
                                                             laserCloudSurfLast->points[minPointInd3].z);

                                LidarPlaneFactor *cost_function = new LidarPlaneFactor(curr_point, last_point_a, last_point_b, last_point_c, 1.0, w_f2f_flat);
                                ceres::internal::ResidualBlock *res_id =
                                    problem.AddResidualBlock(cost_function, loss_function, para_Pose[frameCount], para_Pose[frameCount + 1], para_Ex_Pose[0]);
                                res_ids_f2f_plane.push_back(res_id);

                                if (frameCount == 0) {
                                    //add first window residual block for marginalization
                                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(cost_function, loss_function,
                                                                                               vector<double *>{para_Pose[frameCount], para_Pose[frameCount + 1], para_Ex_Pose[0]},
                                                                                               vector<int>{0});
                                    f2f_block.push(residual_block_info);
                                }
                            }
                        }
                    }
                }
            }
            if (ENABLE_F2F_EDGE){
                if(frame_i->second.laserCloudCornerStackNum > 0 && frameCount != SWEEP_SIZE){
                    auto frame_j = std::next(frame_i);
                    laserCloudCornerLast = frame_i->second.laserCloudCornerStack;
                    Feature_All->kdtreeCornerLast->setInputCloud(laserCloudCornerLast);

                    // trans j lidar frame point into i lidar frame
                    ncrl_tf::Trans Tw2lj, Tw2li, Tli2lj, tmpExtrinisc;
                    ncrl_tf::setTransFrame(Tw2lj, "WORLD", "LIDAR_J");
                    ncrl_tf::setTransFrame(Tw2li, "WORLD", "LIDAR_I");
                    ncrl_tf::setTransFrame(Tli2lj, "LIDAR_I", "LIDAR_J");

                    tmpExtrinisc = EXTRINSIC;
                    ncrl_tf::setTransFrame(tmpExtrinisc, "IMU", "LIDAR_J");
                    if (!ncrl_tf::accumTrans(Tw2lj, frame_j->second.Tw2curr_W, tmpExtrinisc))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    ncrl_tf::setTransFrame(tmpExtrinisc, "IMU", "LIDAR_I");
                    if (!ncrl_tf::accumTrans(Tw2li, frame_i->second.Tw2curr_W, tmpExtrinisc))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    if (!ncrl_tf::deltaTrans(Tli2lj, Tw2li, Tw2lj))
                        ROS_WARN("TRANSFORMATION IN FRAME TO FRAME PREPROCESS IS WRONG");

                    for (int i = 0; i < (int)frame_j->second.laserCloudCornerStackNum; i++)
                    {
                        PointType pointOri;
                        PointType pointSel;
                        std::vector<int> pointSearchInd;
                        std::vector<float> pointSearchSqDis;
                        pointOri = frame_j->second.laserCloudCornerStack->points[i];

                        Feature_All->pointAssociateToMap(&pointOri, &pointSel, Tli2lj.q, Tli2lj.v);
                        Feature_All->kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            if (Feature_All->LidarF2fEdgeHandler(laserCloudCornerLast, pointSel, closestPointInd, minPointInd2))
                            {
                                Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                                Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                             laserCloudCornerLast->points[closestPointInd].y,
                                                             laserCloudCornerLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                             laserCloudCornerLast->points[minPointInd2].y,
                                                             laserCloudCornerLast->points[minPointInd2].z);

                                LidarEdgeIJFactor *cost_function = new LidarEdgeIJFactor(curr_point, last_point_a, last_point_b, 1.0, w_f2f_corner);
                                ceres::internal::ResidualBlock *res_id =
                                    problem.AddResidualBlock(cost_function, loss_function,  para_Pose[frameCount], para_Pose[frameCount + 1], para_Ex_Pose[0]);
                                res_ids_f2f_edge.push_back(res_id);

                                if(frameCount == 0){
                                    //add first window residual block for marginalization
                                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(cost_function, loss_function,
                                                                                               vector<double *>{para_Pose[frameCount], para_Pose[frameCount + 1], para_Ex_Pose[0]},
                                                                                               vector<int>{0});
                                    f2f_block.push(residual_block_info);
                                }
                            }
                        }
                    }
                }
            }
          // feature point is not enough
        } else {
            enoughSolved = false;
            ROS_WARN("FEATURE POINTS ARE NOT ENOUGH ON %d !", frameCount);
        }
        frameCount++;
    }// for (frame_i = Feature_All->Feature_All_Map.begin(); frame_i != Feature_All->Feature_All_Map.end(); frame_i++)

    //residual before optimization
    ceres::Problem::EvaluateOptions b_option;
    b_option.parameter_blocks = para_ids;
    b_option.residual_blocks = res_ids_imu;
    problem.Evaluate(b_option, &C_imu.prev, NULL, NULL, NULL);
    if (ENABLE_F2M_EDGE){
        b_option.residual_blocks = res_ids_edge;
        problem.Evaluate(b_option, &C_F2M_E.prev, NULL, NULL, NULL);
    }
    if (ENABLE_F2M_PLANE){
        b_option.residual_blocks = res_ids_plane;
        problem.Evaluate(b_option, &C_F2M_P.prev, NULL, NULL, NULL);
    }
    if (ENABLE_F2F_PLANE){
        b_option.residual_blocks = res_ids_f2f_plane;
        problem.Evaluate(b_option, &C_F2F_P.prev, NULL, NULL, NULL);
    }
    if (ENABLE_F2F_EDGE){
        b_option.residual_blocks = res_ids_f2f_edge;
        problem.Evaluate(b_option, &C_F2F_E.prev, NULL, NULL, NULL);
    }
    if (ENABLE_MARGINALIZATION && last_marginalization_info && !res_ids_marg.empty()){
        b_option.residual_blocks = res_ids_marg;
        problem.Evaluate(b_option, &C_Marginal.prev, NULL, NULL, NULL);
    }

    // cere solver solve
    if (enoughSolved){
        TicToc t_cere_solve;
        ceres::Solver::Options options;
//        options.linear_solver_type = ceres::DENSE_QR;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = NUM_ITERATIONS;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        options.max_solver_time_in_seconds = SOLVER_TIME;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        t_cereSolver = t_cere_solve.toc();

        //residual after optimization
        b_option.parameter_blocks = para_ids;
        b_option.residual_blocks = res_ids_imu;
        problem.Evaluate(b_option, &C_imu.after, NULL, NULL, NULL);
        if (ENABLE_F2M_EDGE) {
            b_option.residual_blocks = res_ids_edge;
            problem.Evaluate(b_option, &C_F2M_E.after, NULL, NULL, NULL);
        }
        if (ENABLE_F2M_PLANE) {
            b_option.residual_blocks = res_ids_plane;
            problem.Evaluate(b_option, &C_F2M_P.after, NULL, NULL, NULL);
        }
        if (ENABLE_F2F_PLANE) {
            b_option.residual_blocks = res_ids_f2f_plane;
            problem.Evaluate(b_option, &C_F2F_P.after, NULL, NULL, NULL);
        }
        if (ENABLE_F2F_EDGE) {
            b_option.residual_blocks = res_ids_f2f_edge;
            problem.Evaluate(b_option, &C_F2F_E.after, NULL, NULL, NULL);
        }
        if (ENABLE_MARGINALIZATION && last_marginalization_info && !res_ids_marg.empty()) {
            b_option.residual_blocks = res_ids_marg;
            problem.Evaluate(b_option, &C_Marginal.after, NULL, NULL, NULL);
        }
  //      ROS_WARN("feature_distace: %f, %s", feature_distance, marginalization_flag ? "Non-keyframe" : "Keyframe");
    } else {
        ROS_WARN("INITIAL STATE CANNOT SOLVE");
    }

    // para and [state]s all on world frame
    double2vector();

    // update other odometry state and update map
    frameCount = 0;
    for (frame_i = Feature_All->Feature_All_Map.begin(); frame_i != Feature_All->Feature_All_Map.end(); frame_i++){
        Eigen::Quaterniond q_(para_Pose[frameCount][6], para_Pose[frameCount][3], para_Pose[frameCount][4], para_Pose[frameCount][5]);
        Eigen::Vector3d v_(para_Pose[frameCount][0], para_Pose[frameCount][1], para_Pose[frameCount][2]);
        ncrl_tf::setTrans(frame_i->second.Tw2curr_W, q_, v_);
        if(!ncrl_tf::deltaTrans(frame_i->second.Tw2curr_I, WORLD2IMU, frame_i->second.Tw2curr_W))
            ROS_WARN("Trans world to imu -> imu_init to imu fail");
        if(!ncrl_tf::TransOdometry(EXTRINSIC, frame_i->second.Tw2curr_I, frame_i->second.Tw2curr_L))
            ROS_WARN("Trans imu_init to imu -> lidar_init to lidar fail");
        if(!ncrl_tf::deltaTrans(frame_i->second.Tcurr2odom_I, frame_i->second.Tw2curr_I, frame_i->second.Tinit2odom_st_I))
            ROS_WARN("Delta curr 2 odom on imu is fail");
        if(!ncrl_tf::deltaTrans(frame_i->second.Tcurr2odom_L, frame_i->second.Tw2curr_L, frame_i->second.Tinit2odom_st_L))
            ROS_WARN("Delta curr 2 odom on lidar is fail");

        frameCount++;
        // visualization map
        if (frameCount == SWEEP_SIZE + 1){
            Feature_All->laserCloudSurround->clear();
            for (int i = 0; i < 4851; i++){
                *Feature_All->laserCloudSurround += *Feature_All->laserCloudCornerArray[i];
                *Feature_All->laserCloudSurround += *Feature_All->laserCloudSurfArray[i];
            }
        }
    }

    // Construct Map
    Feature_All->UpdateMap();

    // marginal oldest window as marginalization constrains
    marginalization();
}

void ncrl_lio_estimator_new::marginalization(){
    if (ENABLE_MARGINALIZATION) {
        if (marginalization_flag == MARGIN_OLD) {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();

            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                        last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            {
                std::map<int, FeaturePerSweep>::iterator sweep_it;
                int i = 0;
                for (sweep_it = Feature_All->Feature_All_Map.begin(); sweep_it!= Feature_All->Feature_All_Map.end(); sweep_it++, i++){
                    if (i == 1 && sweep_it->second.pre_integration->sum_dt < 10.0){
                        IMUFactor* imu_factor = new IMUFactor(sweep_it->second.pre_integration);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                                   vector<double *>{para_Pose[0], para_SpeedBias[0],
                                                                                                    para_Pose[1], para_SpeedBias[1]},
                                                                                   vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }

            {
                while(!f2f_block.empty()){
                    marginalization_info->addResidualBlockInfo(f2f_block.front());
                    f2f_block.pop();
                }
            }

            marginalization_info->preMarginalize();
            marginalization_info->marginalize();

            std::unordered_map<long, double *> addr_shift;
            for (int i = 1; i <= SWEEP_SIZE; i++)
            {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            }
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[0])] = para_Ex_Pose[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                last_marginalization_info = nullptr;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
}
