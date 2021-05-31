#include "ncrl_lio/ncrl_lio_estimator_node_fusion.h"
#include "ncrl_lio/ncrl_pose_graph.h"
#define VISUALIZE

Eigen::Matrix4f Initial_guess;
pcl::PointCloud<PointType>::Ptr Map_Cloud(new pcl::PointCloud<PointType>());
int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
const float cubeSize = 100;
const float cubeSizeHalf = cubeSize / 2.0;
pcl::PointCloud<PointType>::Ptr laserCloudArray[4851];
nav_msgs::Path global_path;
nav_msgs::Path local_path;

int ICP_count = 0;
double T_ICP_sum = 0.0;
double T_keyframe_sum = 0.0;
int optimize_count = 0;
double T_optimize_sum = 0.0;
double LastLoopTime = 0.0;
int LastLoopID = 0;
int UpdateStartID = 0;
int UpdateEndID = 0;

bool Loop_Closed = false;
bool System_Init = false;
ncrl_tf::Trans T_lastkeyframe;
ncrl_tf::Trans T_accumulate;
pcl::PointCloud<PointType>::Ptr KeyPose_Cloud3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr KeyPose_Cloud6D(new pcl::PointCloud<PointTypePose>());
std::vector<pcl::PointCloud<PointType>::Ptr> Key_LocalCloud;
pcl::KdTreeFLANN<PointType>::Ptr KdtreeKeyPoses(new pcl::KdTreeFLANN<PointType>());

std::queue<nav_msgs::OdometryConstPtr> laserOdomBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> mapPointsBuf;
std::mutex m_buf;
std::mutex m_process;

ros::Publisher pub_gloabl_path;
ros::Publisher pub_local_path;
ros::Publisher pub_map;
ros::Publisher pub_align;
ros::Publisher pub_relo;
ros::Publisher pub_match;

void PassThroughFilter(Eigen::Vector3d current_pos, pcl::PointCloud<PointType>::Ptr local_cloud,
                       double radius, pcl::PointCloud<PointType>::Ptr &cloud_filtered){
    // filter the pointcloud within the radius of the current position
    pcl::PassThrough<PointType> passx;
    passx.setInputCloud (local_cloud);
    passx.setFilterFieldName ("x");
    passx.setFilterLimits (current_pos.x() - radius, current_pos.x() + radius);
    passx.filter (*cloud_filtered);
    pcl::PassThrough<PointType> passy;
    passy.setInputCloud (cloud_filtered);
    passy.setFilterFieldName ("y");
    passy.setFilterLimits (current_pos.y() - radius, current_pos.y() + radius);
    passy.filter (*cloud_filtered);
}

void SavePoseGraph()
{
    FILE *pFile;
    string file_path = OUTPUT_PATH + "loop.txt";
    ROS_WARN_STREAM("Add " << KeyPose_Cloud6D->points.size() << " pose to " << file_path);
    // add keyframe pose to file
    pFile = fopen (file_path.c_str(), "w");

    for (int i = 0; i < (int)KeyPose_Cloud6D->points.size(); i++) {
        fprintf (pFile, "%.18e %.18e %.18e %.18e %.18e %.18e %.18e %.18e\n", KeyPose_Cloud6D->points[i].time,
                 KeyPose_Cloud6D->points[i].x, KeyPose_Cloud6D->points[i].y, KeyPose_Cloud6D->points[i].z,
                 KeyPose_Cloud6D->points[i].qx, KeyPose_Cloud6D->points[i].qy,
                 KeyPose_Cloud6D->points[i].qz, KeyPose_Cloud6D->points[i].qw);
    }
    fclose(pFile);
}

void LaserOdomHandler(const nav_msgs::OdometryConstPtr &msg){
    m_buf.lock();
    laserOdomBuf.push(msg);
    m_buf.unlock();
}

void MapHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    m_buf.lock();
    mapPointsBuf.push(msg);
    m_buf.unlock();
}

void GetMeasurements(std_msgs::Header &header, pcl::PointCloud<PointType>::Ptr &local_cloud,
                     nav_msgs::OdometryConstPtr &local_pose){
    if (!mapPointsBuf.empty() && !laserOdomBuf.empty()) {
        double timeOdom = laserOdomBuf.front()->header.stamp.toSec();
        double timeMap = mapPointsBuf.front()->header.stamp.toSec();
        // Get the measurement with the same timestamp
        if (timeMap != timeOdom) {
            if (timeMap > timeOdom)
              laserOdomBuf.pop();
            else if (timeMap < timeOdom)
              mapPointsBuf.pop();
        } else {
            local_pose = laserOdomBuf.front();
            laserOdomBuf.pop();
            sensor_msgs::PointCloud2ConstPtr map_msg = mapPointsBuf.front();
            header = map_msg->header;
            mapPointsBuf.pop();
            pcl::fromROSMsg(*map_msg, *local_cloud);
        }
    }
}

void TransformToWorld(PointType const *const pi, PointType *const po,
                      Eigen::Quaterniond q_w_curr, Eigen::Vector3d t_w_curr){
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

void TransformToBody(PointType const *const pi, PointType *const po,
                     Eigen::Quaterniond q_w_curr, Eigen::Vector3d t_w_curr){
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_b = q_w_curr.inverse() * (point_curr - t_w_curr);
    po->x = point_b.x();
    po->y = point_b.y();
    po->z = point_b.z();
    po->intensity = pi->intensity;
}

bool DetectLoopClosure(PointType current_3D, PointTypePose current_6D, int &ClosestID)
{
    // find the closest history keyframe
    std::vector<int> KeyframeIDs;
    std::vector<float> KeyframeSqDis;
    KdtreeKeyPoses->setInputCloud(KeyPose_Cloud3D);

    // compute searching radius by 2% of total distance
    double radius = (KeyPose_Cloud6D->points.size() - LastLoopID) * KeyframeDistance * 0.02;
//    ROS_WARN("radius: %f, %lu, %d", radius, KeyPose_Cloud6D->points.size(), LastLoopID);
    if (radius < LoopSearchRadius)
      radius = LoopSearchRadius;
    KdtreeKeyPoses->radiusSearch(current_3D, radius, KeyframeIDs, KeyframeSqDis, 0);

    ClosestID = -1;
    for (int i = 0; i < (int)KeyframeIDs.size(); i++) {
        int ID = KeyframeIDs[i];
        if (abs(KeyPose_Cloud6D->points[ID].time - current_6D.time) > MinLoopDuration) {
            ClosestID = ID;
            break;
        }
    }
    if (ClosestID == -1)
      return false;
    else
      return true;
}

bool Compute_ICP(pcl::PointCloud<PointType>::Ptr local_cloud, pcl::PointCloud<PointType>::Ptr Map_Cloud,
                 ncrl_tf::Trans T_init, pcl::PointCloud<PointType>::Ptr &align_cloud, double &var, double score_thr)
{
    TicToc t_icp;
    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(local_cloud);
    icp.setInputTarget(Map_Cloud);

    // set initial guess
#if 1
      Eigen::Vector3f t_shift(0, 0, 0);
#else
      Eigen::Vector3f t_shift(float(T_init.v.x()), float(T_init.v.y()), float(T_init.v.z()));
#endif
    Eigen::Vector3d q_shift = Q2RPY(T_init.q);
    Eigen::Vector3f r_shift(float(q_shift.z()), float(q_shift.y()), float(q_shift.x()));
    Initial_guess.block<3, 1>(0, 3) = t_shift;
    Initial_guess.block<3, 3>(0, 0) = ypr2R(r_shift);
    icp.align(*align_cloud, Initial_guess);

    ROS_DEBUG("icp costs: %fms, score: %f", t_icp.toc(), icp.getFitnessScore());
    T_ICP_sum += t_icp.toc();
    ICP_count ++;
    ROS_DEBUG("T_ICP avg costs: %fms", T_ICP_sum/ICP_count);
    if (icp.hasConverged()) {
        Initial_guess = icp.getFinalTransformation();
        double current_score = icp.getFitnessScore();
        if (current_score < score_thr) {
            var = sqrt(current_score);
            return true;
        }
        else
          return false;
    }
    else
      return false;
}

void UpdateKeyframePath(PointTypePose current_6D){
    geometry_msgs::PoseStamped pos_path_msg;
    pos_path_msg.header.stamp = ros::Time(current_6D.time);
    pos_path_msg.header.frame_id = "WORLD";
    pos_path_msg.pose.position.x = current_6D.x;
    pos_path_msg.pose.position.y = current_6D.y;
    pos_path_msg.pose.position.z = current_6D.z;
    pos_path_msg.pose.orientation.x = current_6D.qx;
    pos_path_msg.pose.orientation.y = current_6D.qy;
    pos_path_msg.pose.orientation.z = current_6D.qz;
    pos_path_msg.pose.orientation.w = current_6D.qw;
    global_path.poses.push_back(pos_path_msg);
}

void UpdateKeyPose(double t_array[][3], double euler_array[][3], pcl::PointCloud<PointTypePose>::Ptr KeyPose_tmp){
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < (int)KeyPose_tmp->points.size(); i++) {
        Eigen::Quaterniond tmp_q;
        tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);

        PointType current_3D;
        current_3D.x = tmp_t.x();
        current_3D.y = tmp_t.y();
        current_3D.z = tmp_t.z();
        KeyPose_Cloud3D->points.push_back(current_3D);

        PointTypePose current_6D;
        current_6D.x = tmp_t.x();
        current_6D.y = tmp_t.y();
        current_6D.z = tmp_t.z();
        current_6D.qx = tmp_q.x();
        current_6D.qy = tmp_q.y();
        current_6D.qz = tmp_q.z();
        current_6D.qw = tmp_q.w();
        current_6D.time = KeyPose_tmp->points[i].time;
        current_6D.loopID = KeyPose_tmp->points[i].loopID;
        current_6D.loop_tx = KeyPose_tmp->points[i].loop_tx;
        current_6D.loop_ty = KeyPose_tmp->points[i].loop_ty;
        current_6D.loop_tz = KeyPose_tmp->points[i].loop_tz;
        current_6D.loop_yaw = KeyPose_tmp->points[i].loop_yaw;
        current_6D.var = KeyPose_tmp->points[i].var;
        KeyPose_Cloud6D->points.push_back(current_6D);
#ifdef VISUALIZE
        UpdateKeyframePath(current_6D);
        // add arrow between current and history node
        if (current_6D.loopID != -1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "WORLD";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.5; marker.scale.y = 1.0;
            marker.color.b = 1; marker.color.g = 0; marker.color.r = 0;
            marker.color.a = 1; marker.id = i;

            geometry_msgs::Point p_;
            p_.x = tmp_t.x();
            p_.y = tmp_t.y();
            p_.z = tmp_t.z();
            marker.points.push_back(p_);
            geometry_msgs::Point p;
            p.x = KeyPose_Cloud6D->points[current_6D.loopID].x;
            p.y = KeyPose_Cloud6D->points[current_6D.loopID].y;
            p.z = KeyPose_Cloud6D->points[current_6D.loopID].z;
            marker.points.push_back(p);
            markers.markers.push_back(marker);
        }
#endif
    }
    if (markers.markers.size() > 0)
      pub_match.publish(markers);
}

void UpdateGlobalMap(int start, int end){
    for (int i = start; i < end; i++) {
        // build the global map according to the keyframe poses
        Eigen::Vector3d w_P_cur(KeyPose_Cloud6D->points[i].x, KeyPose_Cloud6D->points[i].y, KeyPose_Cloud6D->points[i].z);
        Eigen::Quaterniond w_Q_cur(KeyPose_Cloud6D->points[i].qw, KeyPose_Cloud6D->points[i].qx,
                                   KeyPose_Cloud6D->points[i].qy, KeyPose_Cloud6D->points[i].qz);
        for (int j = 0; j < (int)Key_LocalCloud[i]->points.size(); j++) {
            PointType pointSel;
            PointType pointOri = Key_LocalCloud[i]->points[j];
            TransformToWorld(&pointOri, &pointSel, w_Q_cur, w_P_cur);
#ifdef VISUALIZE
            int cubeI = int((pointSel.x + cubeSizeHalf) / cubeSize) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + cubeSizeHalf) / cubeSize) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + cubeSizeHalf) / cubeSize) + laserCloudCenDepth;

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
                laserCloudArray[cubeInd]->push_back(pointSel);
            }
#endif
        }
    }
}

void optimize4DoF(){
      // declare states for ceres
      double t_array[KeyPose_Cloud6D->points.size()][3];
      Eigen::Quaterniond q_array[KeyPose_Cloud6D->points.size()];
      double euler_array[KeyPose_Cloud6D->points.size()][3];

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      //options.minimizer_progress_to_stdout = true;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction *loss_function;
      loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

      // construct cost function
      for (int i = 0; i < (int)KeyPose_Cloud6D->points.size(); i++) {
          // set initial guess from LIO
          Eigen::Quaterniond tmp_q(KeyPose_Cloud6D->points[i].qw, KeyPose_Cloud6D->points[i].qx,
                                   KeyPose_Cloud6D->points[i].qy, KeyPose_Cloud6D->points[i].qz);
          t_array[i][0] = KeyPose_Cloud6D->points[i].x;
          t_array[i][1] = KeyPose_Cloud6D->points[i].y;
          t_array[i][2] = KeyPose_Cloud6D->points[i].z;
          q_array[i] = tmp_q;

          Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
          euler_array[i][0] = euler_angle.x();
          euler_array[i][1] = euler_angle.y();
          euler_array[i][2] = euler_angle.z();

          problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
          problem.AddParameterBlock(t_array[i], 3);
          if (i == 0) {
              problem.SetParameterBlockConstant(euler_array[i]);
              problem.SetParameterBlockConstant(t_array[i]);
          }
          // add edge with previous node
          for (int j = 1; j < 5; j++)
          {
            if (i - j >= 0)
            {
              Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
              Eigen::Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1],
                                  t_array[i][2] - t_array[i-j][2]);
              // transform relative translation to i frame
              relative_t = q_array[i-j].inverse() * relative_t;
              double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
              ceres::CostFunction* cost_function = FourDOFError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
              problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
                                       t_array[i-j], euler_array[i], t_array[i]);
            }
          }

          // add edge with loop node
          int loop_ID = KeyPose_Cloud6D->points[i].loopID;
          if (loop_ID != -1) {
              if (loop_ID >= i)
                ROS_WARN("Wrong loopID %d, current %d!", loop_ID, i);
              else {
                  Eigen::Vector3d current_t(t_array[i][0] - t_array[loop_ID][0], t_array[i][1] - t_array[loop_ID][1],
                                            t_array[i][2] - t_array[loop_ID][2]);
                  current_t = q_array[loop_ID].inverse() * current_t;
                  Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[loop_ID].toRotationMatrix());
                  Eigen::Vector3d relative_t(KeyPose_Cloud6D->points[i].loop_tx, KeyPose_Cloud6D->points[i].loop_ty,
                                             KeyPose_Cloud6D->points[i].loop_tz);
//                  double loop_error = (relative_t - current_t).norm();
//                  ROS_WARN("loop_error %f, first_error %f", loop_error, KeyPose_Cloud6D->points[i].var);
                  ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                       KeyPose_Cloud6D->points[i].loop_yaw, euler_conncected.y(),
                                                       euler_conncected.z(), 1.0);
                  problem.AddResidualBlock(cost_function, loss_function, euler_array[loop_ID],
                                           t_array[loop_ID], euler_array[i], t_array[i]);
              }
          }
      }
      ceres::Solve(options, &problem, &summary);

      // update keyframe pose and the global map
      pcl::PointCloud<PointTypePose>::Ptr KeyPose_tmp(new pcl::PointCloud<PointTypePose>());
      pcl::copyPointCloud(*KeyPose_Cloud6D, *KeyPose_tmp);
      KeyPose_Cloud3D->clear();
      KeyPose_Cloud6D->clear();
      for (int i = 0; i < 4851; i++) {
          laserCloudArray[i]->clear();
      }
      global_path.poses.clear();
      UpdateKeyPose(t_array, euler_array, KeyPose_tmp);
      UpdateGlobalMap(0, UpdateStartID - 1);
      Loop_Closed = false;
}

void process(){
    while(ros::ok()){
        m_buf.lock();
        std_msgs::Header header;
        nav_msgs::OdometryConstPtr local_pose = NULL;
        pcl::PointCloud<PointType>::Ptr local_cloud(new pcl::PointCloud<PointType>());
        GetMeasurements(header, local_cloud, local_pose);
        m_buf.unlock();

        m_process.lock();
        if (!local_cloud->empty() && local_pose != NULL && !Loop_Closed) {
            Eigen::Vector3d current_pos(local_pose->pose.pose.position.x, local_pose->pose.pose.position.y,
                                        local_pose->pose.pose.position.z);
            Eigen::Quaterniond current_ori(local_pose->pose.pose.orientation.w, local_pose->pose.pose.orientation.x,
                                        local_pose->pose.pose.orientation.y, local_pose->pose.pose.orientation.z);
#if 0
            if ((header.stamp.toSec() - LastLoopTime) > 0.9 || !System_Init) {
#else
            // compute position norm with last keyframe
            Eigen::Vector3d diff = current_pos - T_lastkeyframe.v;
            if (diff.norm() > KeyframeDistance || !System_Init) {
#endif
                TicToc t_keyframe;
                pcl::PointCloud<PointType>::Ptr current_cloud(new pcl::PointCloud<PointType>());
                for (int i = 0; i < (int)local_cloud->points.size(); i++) {
                    PointType pointSel = local_cloud->points[i];
                    TransformToBody(&pointSel, &pointSel, current_ori, current_pos);
                    current_cloud->points.push_back(pointSel);
                }
                ncrl_tf::Trans T_current;
                ncrl_tf::setTransFrame(T_current, "WORLD", "CURRENT");
                ncrl_tf::setTrans(T_current, current_ori, current_pos);

                geometry_msgs::PoseStamped pos_msg;
                pos_msg.header = header;
                pos_msg.header.frame_id = "WORLD";
                pos_msg.pose.position.x = current_pos.x();
                pos_msg.pose.position.y = current_pos.y();
                pos_msg.pose.position.z = current_pos.z();
                pos_msg.pose.orientation.x = current_ori.x();
                pos_msg.pose.orientation.y = current_ori.y();
                pos_msg.pose.orientation.z = current_ori.z();
                pos_msg.pose.orientation.w = current_ori.w();
                local_path.poses.push_back(pos_msg);

                ncrl_tf::Trans T_diff;
                ncrl_tf::setTransFrame(T_diff, "LAST", "CURRENT");
                // compute transform form last to current keyframe
                if (!ncrl_tf::deltaTrans(T_diff, T_lastkeyframe, T_current))
                  ROS_WARN("TRANSFORMATION IN T_diff IS WRONG");

                // accumulate transform based on T_diff, T_accumulate will be update after optimization
                ncrl_tf::setTransFrame(T_accumulate, "WORLD", "LAST");
                if (!ncrl_tf::accumTrans(T_current, T_accumulate, T_diff))
                  ROS_WARN("TRANSFORMATION IN T_accumulate IS WRONG");
                ncrl_tf::setTrans(T_accumulate, T_current.q, T_current.v);

                PointType current_3D;
                current_3D.x = T_current.v.x();
                current_3D.y = T_current.v.y();
                current_3D.z = T_current.v.z();
                PointTypePose current_6D;
                current_6D.x = T_current.v.x();
                current_6D.y = T_current.v.y();
                current_6D.z = T_current.v.z();
                current_6D.qx = T_current.q.x();
                current_6D.qy = T_current.q.y();
                current_6D.qz = T_current.q.z();
                current_6D.qw = T_current.q.w();
                current_6D.time = header.stamp.toSec();
                current_6D.loopID = -1;
                current_6D.var = 0;
                UpdateKeyframePath(current_6D);

                ncrl_tf::Trans T_history, T_shift, T_correct, T_drift;
                static tf::TransformBroadcaster br;
                tf::Transform tf_history, tf_current;
                ncrl_tf::setTfTrans(tf_current, T_current.q, T_current.v);
                br.sendTransform(tf::StampedTransform(tf_current, ros::Time::now(), T_current.start_frame, T_current.end_frame));

                // continue if keyframe size is less than 30
                if (KeyPose_Cloud3D->size() < 30) {
                    KeyPose_Cloud3D->push_back(current_3D);
                    KeyPose_Cloud6D->push_back(current_6D);
                    Key_LocalCloud.push_back(current_cloud);
                    ncrl_tf::setTrans(T_lastkeyframe, current_ori, current_pos);
                    System_Init = true;
                    T_keyframe_sum += t_keyframe.toc();

                    m_process.unlock();
                    continue;
                }

                // the loop will only detect every interval
                bool Detect_Loop = false;
                if ((header.stamp.toSec() - LastLoopTime) > 0.0) {
                    Detect_Loop = true;
                    LastLoopTime = header.stamp.toSec();
                }

                int history_ID = -1;
                // loop candidate is searched with KeyPose_Cloud3D,
                // and loop closure information is stored in KeyPose_Cloud6D
                if (Detect_Loop && DetectLoopClosure(current_3D, current_6D, history_ID)) {
                    Eigen::Vector3d history_pos(KeyPose_Cloud6D->points[history_ID].x, KeyPose_Cloud6D->points[history_ID].y,
                                                KeyPose_Cloud6D->points[history_ID].z);
                    Eigen::Quaterniond history_ori(KeyPose_Cloud6D->points[history_ID].qw, KeyPose_Cloud6D->points[history_ID].qx,
                                                   KeyPose_Cloud6D->points[history_ID].qy, KeyPose_Cloud6D->points[history_ID].qz);
                    ncrl_tf::setTransFrame(T_history, "WORLD", "HISTORY");
                    ncrl_tf::setTrans(T_history, history_ori, history_pos);
                    ncrl_tf::setTfTrans(tf_history, T_history.q, T_history.v);
                    br.sendTransform(tf::StampedTransform(tf_history, ros::Time::now(), T_history.start_frame, T_history.end_frame));

                    // filter Map_Cloud around history_pos and transform cloud to history frame
                    pcl::PointCloud<PointType>::Ptr history_cloud(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr Candidate_Cloud(new pcl::PointCloud<PointType>());
                    PassThroughFilter(history_pos, Map_Cloud, 80, history_cloud);
                    for (int i = 0; i < (int)history_cloud->points.size(); i++) {
                        PointType pointSel = history_cloud->points[i];
                        TransformToBody(&pointSel, &pointSel, T_history.q, T_history.v);
                        Candidate_Cloud->points.push_back(pointSel);
                    }
                    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
                    PassThroughFilter(Eigen::Vector3d(0, 0, 0), current_cloud, 60, cloud_filtered);
                    if (Candidate_Cloud->empty()) {
                        ROS_WARN("Traget Cloud for ICP is empty!");
                        m_process.unlock();
                        continue;
                    }
                    if (cloud_filtered->empty()) {
                        ROS_WARN("Source Cloud for ICP is empty!");
                        m_process.unlock();
                        continue;
                    }

                    // compute initial guess for ICP
                    ncrl_tf::Trans T_history_current;
                    if (!ncrl_tf::deltaTrans(T_history_current, T_history, T_current))
                      ROS_WARN("TRANSFORMATION IN T_current_history IS WRONG");

                    // perform loop closure with ICP, pose graph optimization is done in optimize4DoF
                    double var_ = -1;
                    pcl::PointCloud<PointType>::Ptr align_cloud(new pcl::PointCloud<PointType>());
                    if (Compute_ICP(cloud_filtered, Candidate_Cloud, T_history_current, align_cloud, var_, ICP_Score)) {
                        Eigen::Vector3d t_shift(double(Initial_guess(0, 3)), double(Initial_guess(1, 3)),
                                                double(Initial_guess(2, 3)));
                        Eigen::Matrix3f R_tmp = Initial_guess.block<3, 3>(0, 0);
                        Eigen::Vector3f ypr_tmp = R2ypr(R_tmp);
                        Eigen::Vector3d rpy_tmp(double(ypr_tmp.z()), double(ypr_tmp.y()), double(ypr_tmp.x()));
                        Eigen::Quaterniond q_shift = ncrl_tf::Euler2Q(rpy_tmp);
#if 1
                        // compute relative transform between current and history node
                        ncrl_tf::setTransFrame(T_shift, "HISTORY", "CORRECT");
                        ncrl_tf::setTrans(T_shift, q_shift, t_shift);
                        if (!ncrl_tf::accumTrans(T_correct, T_history, T_shift))
                          ROS_WARN("TRANSFORMATION IN T_correct IS WRONG");
                        if (!ncrl_tf::deltaTrans(T_drift, T_current, T_correct))
                          ROS_WARN("TRANSFORMATION IN T_drift IS WRONG");
                        ROS_WARN_STREAM("t_drift " << T_drift.v.transpose() <<
                                        ", q_drift " << rad2deg(Q2RPY(T_drift.q)).transpose());
#endif
                        // update loop closure information,
                        // var will affect the covariance of loop node
                        current_6D.loopID = history_ID;
                        current_6D.loop_tx = t_shift.x();
                        current_6D.loop_ty = t_shift.y();
                        current_6D.loop_tz = t_shift.z();
                        current_6D.loop_yaw = double(ypr_tmp.x());
                        current_6D.var = 1.0;
//                            current_6D.var = T_drift.v.norm();
                        Loop_Closed = true;
                        LastLoopID = KeyPose_Cloud6D->points.size();
                    }
                    else
                      ROS_DEBUG("Loop closure is not perform!");
#ifdef VISUALIZE
                    sensor_msgs::PointCloud2 matched_cloud;
                    pcl::toROSMsg(*align_cloud, matched_cloud);
                    matched_cloud.header.stamp = ros::Time::now();
                    matched_cloud.header.frame_id = "HISTORY";
                    pub_align.publish(matched_cloud);
#endif
                } else {
                    ROS_DEBUG("Waiting for loop node......");
                }
                // store data into database and update T_lastkeyframe for next iteration
                KeyPose_Cloud3D->push_back(current_3D);
                KeyPose_Cloud6D->push_back(current_6D);
                Key_LocalCloud.push_back(current_cloud);
                ncrl_tf::setTrans(T_lastkeyframe, current_ori, current_pos);
                T_keyframe_sum += t_keyframe.toc();
                ROS_DEBUG("average keyframe costs: %fms", T_keyframe_sum / KeyPose_Cloud6D->points.size());
            }
        }

        if (KeyPose_Cloud6D->points.size() > 30 && Loop_Closed) {
            TicToc t_optimize;
            optimize4DoF();

            int ID = KeyPose_Cloud6D->points.size() - 1;
            Eigen::Vector3d last_pos(KeyPose_Cloud6D->points[ID].x, KeyPose_Cloud6D->points[ID].y,
                                        KeyPose_Cloud6D->points[ID].z);
            Eigen::Quaterniond last_ori(KeyPose_Cloud6D->points[ID].qw, KeyPose_Cloud6D->points[ID].qx,
                                           KeyPose_Cloud6D->points[ID].qy, KeyPose_Cloud6D->points[ID].qz);
            ncrl_tf::setTrans(T_accumulate, last_ori, last_pos);

            ROS_DEBUG("pose graph optimize costs: %fms", t_optimize.toc());
            T_optimize_sum += t_optimize.toc();
            optimize_count++;
            ROS_DEBUG("average optimize costs: %fms", T_optimize_sum / optimize_count);
        } else if ((int)KeyPose_Cloud6D->points.size() > UpdateEndID && KeyPose_Cloud6D->points.size() % 10 == 0) {
            // get UpdateEndID to update the global map
            double tmp_time = KeyPose_Cloud6D->points[KeyPose_Cloud6D->points.size() - 1].time;
            for (int i = KeyPose_Cloud6D->points.size() - 1; i > 0; i--) {
                if (tmp_time - KeyPose_Cloud6D->points[i].time > MinLoopDuration) {
                    UpdateEndID = i;
                    break;
                }
            }
            if (UpdateStartID < UpdateEndID) {
                UpdateGlobalMap(UpdateStartID, UpdateEndID + 1);
                UpdateStartID = UpdateEndID + 1;
                UpdateEndID = KeyPose_Cloud6D->points.size();

                Map_Cloud->clear();
                for (int i = 0; i < 4851; i++) {
                    *Map_Cloud += *laserCloudArray[i];
                }
#ifdef VISUALIZE
                sensor_msgs::PointCloud2 input_map;
                pcl::toROSMsg(*Map_Cloud, input_map);
                if (input_map.data.size() > 0 && pub_map.getNumSubscribers() != 0) {
                    input_map.header.stamp = ros::Time::now();
                    input_map.header.frame_id = "WORLD";
                    pub_map.publish(input_map);
                }
#endif
            }
        }
#ifdef VISUALIZE
        if (global_path.poses.size() > 0 && pub_gloabl_path.getNumSubscribers() != 0) {
            global_path.header.stamp = ros::Time::now();
            global_path.header.frame_id = "WORLD";
            pub_gloabl_path.publish(global_path);
        }
        if (local_path.poses.size() > 0 && pub_local_path.getNumSubscribers() != 0) {
            local_path.header.stamp = ros::Time::now();
            local_path.header.frame_id = "WORLD";
            pub_local_path.publish(local_path);
        }
#endif
        m_process.unlock();

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while (ros::ok())
    {
        char c = getchar();
        if (c == 's') {
            SavePoseGraph();
            UpdateGlobalMap(UpdateStartID, UpdateEndID);
#if 1
            pcl::PointCloud<PointType>::Ptr Edge_Cloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr Plane_Cloud(new pcl::PointCloud<PointType>());
            for (int i = 0; i < (int)Map_Cloud->points.size(); i++) {
                PointType pointSel = Map_Cloud->points[i];
                if (pointSel.intensity < -949 && pointSel.intensity > -1001)
                  Edge_Cloud->push_back(pointSel);
                else if (pointSel.intensity < 51 && pointSel.intensity > -1)
                  Plane_Cloud->push_back(pointSel);
                else
                  ROS_WARN("wrong feature type!!!");
            }
            if (Edge_Cloud->points.size() > 0) {
                if (pcl::io::savePCDFileASCII (OUTPUT_PATH + "map_edge.pcd", *Edge_Cloud))
                  ROS_WARN("Save map_edge failed!");
                else
                  ROS_WARN("Save map_edge success, contain %lu points!", Edge_Cloud->points.size());
            }
            if (Plane_Cloud->points.size() > 0) {
                if (pcl::io::savePCDFileASCII (OUTPUT_PATH + "map_plane.pcd", *Plane_Cloud))
                  ROS_WARN("Save map_plane failed!");
                else
                  ROS_WARN("Save map_plane success, contain %lu points!", Plane_Cloud->points.size());
            }
#else
            if (Map_Cloud->points.size() > 0) {
                if (pcl::io::savePCDFileASCII (OUTPUT_PATH + "map.pcd", *Map_Cloud))
                  ROS_WARN("Save map failed!");
                else
                  ROS_WARN("Save map success, contain %lu points!", Map_Cloud->points.size());
            }
#endif
        }
        std::chrono::milliseconds dura(1000);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ncrl_pose_graph_node");
    ros::NodeHandle nh("~");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(nh);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/estimator/static", 100, MapHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
                                       ("/estimator/laser_odom", 100, LaserOdomHandler);
#ifdef VISUALIZE
    pub_local_path = nh.advertise<nav_msgs::Path> ("/pose_graph/local_path", 10);
    pub_align = nh.advertise<sensor_msgs::PointCloud2> ("/pose_graph/align_cloud", 10);
    pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/pose_graph/map_cloud", 10);
    pub_match = nh.advertise<visualization_msgs::MarkerArray> ("/pose_graph/keyframe_link", 10);
#endif
    pub_gloabl_path = nh.advertise<nav_msgs::Path> ("/pose_graph/keyframe_path", 10);
    pub_relo = nh.advertise<sensor_msgs::PointCloud2> ("/pose_graph/relo_cloud", 10);

    Initial_guess = Eigen::MatrixXf::Identity(4, 4);
    for (int i = 0; i < laserCloudWidth * laserCloudHeight * laserCloudDepth; i++)
      laserCloudArray[i].reset(new pcl::PointCloud<PointType>());

    ROS_INFO_STREAM("ICP_Score " << ICP_Score << ", KeyframeDistance " << KeyframeDistance <<
                    ", LoopSearchRadius " << LoopSearchRadius << ", MinLoopDuration " << MinLoopDuration);

    ncrl_tf::setTransFrame(T_lastkeyframe, "WORLD", "LAST");
    ncrl_tf::setTrans(T_lastkeyframe, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    ncrl_tf::setTransFrame(T_accumulate, "WORLD", "LAST");
    ncrl_tf::setTrans(T_accumulate, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());

    std::thread measurement_process{process};
    std::thread command_process{command};

    ros::spin();
    return 0;
}
