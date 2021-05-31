#include <ros/ros.h>
#include "ncrl_lio/ncrl_lio_estimator_node_fusion.h"

//ncrl_lio_estimator estimator;
ncrl_lio_estimator_new estimator;

// publisher declaration
ros::Publisher pub_odom; // pub laser odometry with nav_msgs:: Odometry
ros::Publisher pub_odom_est; // pub odom which for error estimate
ros::Publisher pub_map; // pub lidar pointCloud map on world frame
ros::Publisher pub_scan; // pub lidar scan on world frame
ros::Publisher pub_static;

// pub state
ros::Publisher pub_pos; // check for position state
ros::Publisher pub_rot; // check for rotation state
ros::Publisher pub_pos_path; // visual for rviz
nav_msgs::Path pos_path; // msg which published by pub_pos_path
std_msgs::Header lidar_header; // time stamp

// callback msg queue : msg will be stored in these queue until getMeasurement() extract the data
std::queue<sensor_msgs::PointCloud2ConstPtr> edgeLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> flatLastBuf;
std::queue<nav_msgs::OdometryConstPtr> laserOdomBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

ros::Publisher pub_latest_odometry;
ros::Publisher pub_latest_odometry_est;
ros::Publisher pub_vel;
ros::Publisher pub_gyr_bias;
ros::Publisher pub_acc_bias;
ros::Publisher pub_high_freq_path;

ros::Publisher pub_ex;
ros::Publisher pub_T_ex;
ros::Publisher pub_R_ex;

nav_msgs::Path high_freq_path;
// callback msg queue : msg will be stored in these queue until getMeasurement() extract the data
std::queue<sensor_msgs::ImuConstPtr> imu_buf;

double current_time = -1; // current time : update in pre-process of processImu
double latest_time; // current time : update through imu data in predict()

// imu propagate variable, will be changed in two space [imu call back] & [update state]
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_imu = true;
double last_imu_t = 0; // check that imu is disorder or not

// mutex
std::mutex m_buf; // mutex for data input, which confine that queue only can be operate by one thread
std::mutex m_estimator; // do in process
std::mutex m_state; // for tmp_state cause these will be changed with two space [imu call back] & [update state]

int lidar_id = -1; // visual show which lidar data in process
int drop_cnt = 0; // count for how many datas we throwed

// evaluation
//int frameCnt_ = 0;
ncrl_tf::Trans F2FODOM_LAST;
ncrl_tf::Trans LIOODOM_LAST;
ncrl_tf::Trans EST_TRANS;

// when update call then tmp_P, tmp_Q, tmp_V, tmp_Ba, tmp_Bg
// will update from nonlinear solve
void predict(const sensor_msgs::ImuConstPtr &imu_msg) {
    double t = imu_msg->header.stamp.toSec();
    if (init_imu) {
        latest_time = t;
        init_imu = false;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // trans on init imu frame, imu propogation on imu init frame
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g.point;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g.point;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// evaluation variable, input delta transformation which is from "IMU_LAST" to "IMU"
void pubEstResult(std_msgs::Header header, ncrl_tf::Trans delta_tr){
    ncrl_tf::Trans result, TransState_est;
    ncrl_tf::setTransFrame(TransState_est, "GT_INIT", "GT");
    ncrl_tf::setTransFrame(result, "IMU_INIT", "IMU");
    if (!ncrl_tf::accumTrans(result, EST_TRANS, delta_tr))
        ROS_WARN("accum in pubEstResult is fail");
    if (!ncrl_tf::TransOdometry(IMU2GT, result, TransState_est))
        ROS_WARN("TRANS FROM IMU TO GT IN UPDATE FUNCTION IS FAIL");

    // update current estimation
    ncrl_tf::setTrans(EST_TRANS, result.q, result.v);

    nav_msgs::Odometry laserOdom_est_msg;
    laserOdom_est_msg.header = header;
    laserOdom_est_msg.header.frame_id = "GT_INIT";
    laserOdom_est_msg.child_frame_id = "GT";
    laserOdom_est_msg.pose.pose.position.x = TransState_est.v.x();
    laserOdom_est_msg.pose.pose.position.y = TransState_est.v.y();
    laserOdom_est_msg.pose.pose.position.z = TransState_est.v.z();
    laserOdom_est_msg.pose.pose.orientation.x = TransState_est.q.x();
    laserOdom_est_msg.pose.pose.orientation.y = TransState_est.q.y();
    laserOdom_est_msg.pose.pose.orientation.z = TransState_est.q.z();
    laserOdom_est_msg.pose.pose.orientation.w = TransState_est.q.w();
//    frameCnt_++;
//    ROS_WARN("estimate result size is %d", frameCnt_);
    pub_odom_est.publish(laserOdom_est_msg);
}

// when process done, update state from nonlinear solve
void update(std_msgs::Header &header){
    geometry_msgs::PointStamped pos_state, rot_state, vel_state, bias_acc, bias_gyr;

    tmp_P = estimator.Ps[SWEEP_SIZE];
    tmp_Q = estimator.Qs[SWEEP_SIZE];
    tmp_V = estimator.Vs[SWEEP_SIZE];
    tmp_Ba = estimator.Bas[SWEEP_SIZE];
    tmp_Bg = estimator.Bgs[SWEEP_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    latest_time = current_time;

    pos_state.header = header;
    pos_state.header.frame_id = "/WORLD";
    pos_state.point.x = estimator.Ps[SWEEP_SIZE][0];
    pos_state.point.y = estimator.Ps[SWEEP_SIZE][1];
    pos_state.point.z = estimator.Ps[SWEEP_SIZE][2];

    vel_state.header = header;
    vel_state.header.frame_id = "/WORLD";
    vel_state.point.x = estimator.Vs[SWEEP_SIZE][0];
    vel_state.point.y = estimator.Vs[SWEEP_SIZE][1];
    vel_state.point.z = estimator.Vs[SWEEP_SIZE][2];

    bias_acc.header = header;
    bias_acc.header.frame_id = "/IMU";
    bias_acc.point.x = estimator.Bas[SWEEP_SIZE][0];
    bias_acc.point.y = estimator.Bas[SWEEP_SIZE][1];
    bias_acc.point.z = estimator.Bas[SWEEP_SIZE][2];

    bias_gyr.header = header;
    bias_gyr.header.frame_id = "/IMU";
    bias_gyr.point.x = estimator.Bgs[SWEEP_SIZE][0];
    bias_gyr.point.y = estimator.Bgs[SWEEP_SIZE][1];
    bias_gyr.point.z = estimator.Bgs[SWEEP_SIZE][2];

    Eigen::Vector3d temp = Q2RPY(estimator.Qs[SWEEP_SIZE]);
    temp = rad2deg(temp);
    rot_state.header = header;
    rot_state.header.frame_id = "/WORLD";
    rot_state.point.x = temp.x();
    rot_state.point.y = temp.y();
    rot_state.point.z = temp.z();

    geometry_msgs::PoseStamped pos_path_msg;
    pos_path_msg.header = header;
    pos_path_msg.header.frame_id = "WORLD";
    pos_path_msg.pose.position = pos_state.point;
    pos_path_msg.pose.orientation.x = estimator.Qs[SWEEP_SIZE].x();
    pos_path_msg.pose.orientation.y = estimator.Qs[SWEEP_SIZE].y();
    pos_path_msg.pose.orientation.z = estimator.Qs[SWEEP_SIZE].z();
    pos_path_msg.pose.orientation.w = estimator.Qs[SWEEP_SIZE].w();
    pos_path.header = header;
    pos_path.header.frame_id = "WORLD";

    // maintain size of low frequency state path
    if(pos_path.poses.size() >= 10000)
        pos_path.poses.erase(pos_path.poses.begin());
    pos_path.poses.push_back(pos_path_msg);
    pub_pos_path.publish(pos_path);

    nav_msgs::Odometry laserOdom_msg;
    laserOdom_msg.header = header;
    laserOdom_msg.header.frame_id = "WORLD";
    laserOdom_msg.child_frame_id = "IMU";
    laserOdom_msg.pose.pose = pos_path_msg.pose;
    pub_odom.publish(laserOdom_msg);

    pub_pos.publish(pos_state);
    pub_rot.publish(rot_state);
    pub_vel.publish(vel_state);
    pub_acc_bias.publish(bias_acc);
    pub_gyr_bias.publish(bias_gyr);


    if (ENABLE_OPT_EXTRINSIC){
        // publish extrinsic state
        geometry_msgs::PoseStamped imu2lidar;
        imu2lidar.header = header;
        imu2lidar.header.frame_id = "IMU";
        imu2lidar.pose.position.x = estimator.til[0].x();
        imu2lidar.pose.position.y = estimator.til[0].y();
        imu2lidar.pose.position.z = estimator.til[0].z();
        imu2lidar.pose.orientation.x = estimator.qil[0].x();
        imu2lidar.pose.orientation.y = estimator.qil[0].y();
        imu2lidar.pose.orientation.z = estimator.qil[0].z();
        imu2lidar.pose.orientation.w = estimator.qil[0].w();

        geometry_msgs::PointStamped t_ex_msg, r_ex_msg;
        t_ex_msg.header = imu2lidar.header;
        t_ex_msg.point = imu2lidar.pose.position;
        Eigen::Vector3d tmp_r_ex = Q2RPY(estimator.qil[0]);
        tmp_r_ex = rad2deg(tmp_r_ex);
        r_ex_msg.header = imu2lidar.header;
        r_ex_msg.point.x = tmp_r_ex.x();
        r_ex_msg.point.y = tmp_r_ex.y();
        r_ex_msg.point.z = tmp_r_ex.z();

        pub_ex.publish(imu2lidar);
        pub_T_ex.publish(t_ex_msg);
        pub_R_ex.publish(r_ex_msg);
    }

    // prepare imu propogate msg
    std::queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (; !tmp_imu_buf.empty(); tmp_imu_buf.pop()){
        predict(tmp_imu_buf.front());
    }
}

// initial msg's frame here
double tlidarlast = 0;
CombinedData getMeasurements(){
    CombinedData measurements;
    while(true){
      // check that feature point is empty or not
      if (imu_buf.empty() || edgeLastBuf.empty() || flatLastBuf.empty()
                          || laserOdomBuf.empty() || fullPointsBuf.empty())
          return measurements;

      double timeEdge = edgeLastBuf.front()->header.stamp.toSec();
      double timeFlat = flatLastBuf.front()->header.stamp.toSec();
      double timeOdom = laserOdomBuf.front()->header.stamp.toSec();
      double timeFull = fullPointsBuf.front()->header.stamp.toSec();

      // check lidar time is sync, after this part msg from laserOdometry will be same
      if (timeEdge != timeFlat || timeFlat != timeOdom || timeOdom != timeFull) {
          ROS_FATAL("TIME UNSYNC !");
          double lidar_array[4] = {timeEdge, timeFlat, timeOdom, timeFull};
          double oldest_time = *std::min_element(lidar_array, lidar_array + 4);
          if (oldest_time == timeEdge)
              edgeLastBuf.pop();
          if (oldest_time == timeFlat)
              flatLastBuf.pop();
          if (oldest_time == timeOdom)
              laserOdomBuf.pop();
          if (oldest_time == timeFull)
              fullPointsBuf.pop();
          drop_cnt++;
          return measurements;
      }

      // check that imu vector is enough or not, the latest time of imu data should larger than lidar
      if (!(imu_buf.back()->header.stamp.toSec() > timeEdge + TD))
          return measurements;

      // only time of imu front < feature time + td
      if (!(imu_buf.front()->header.stamp.toSec() < timeEdge + TD)) {
          ROS_WARN("throw lidar msg, only should happen at the beginning");

          // formulate f2f odometry from last to current
          nav_msgs::OdometryConstPtr odom_msg = laserOdomBuf.front();
          ncrl_tf::Trans tf_odom_I, tf_odom_L, delta_trans, LIOODOM_LAST_;
          Eigen::Vector3d v_(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
          Eigen::Quaterniond q_(odom_msg->pose.pose.orientation.w,
                                odom_msg->pose.pose.orientation.x,
                                odom_msg->pose.pose.orientation.y,
                                odom_msg->pose.pose.orientation.z);
          ncrl_tf::setTransFrame(tf_odom_L, "LIDAR_INIT", "LIDAR");
          ncrl_tf::setTrans(tf_odom_L, q_, v_);
          ncrl_tf::setTransFrame(tf_odom_I, "IMU_INIT", "IMU");
          if(!ncrl_tf::TransOdometry(EXTRINSIC, tf_odom_L, tf_odom_I))
              ROS_WARN("Trans odom from lidar frame to imu frame fail");
          ncrl_tf::setTransFrame(delta_trans, "IMU_LAST", "IMU");
          ncrl_tf::setTransFrame(LIOODOM_LAST_, "IMU_INIT", "IMU");
          if (!ncrl_tf::deltaTrans(delta_trans, F2FODOM_LAST, tf_odom_I))
              ROS_WARN("DELTA TRANSFORM IS WRONG");

          // special case : when imu messages are missing
          pubEstResult(odom_msg->header, delta_trans);

          // update last
          ncrl_tf::setTrans(F2FODOM_LAST, tf_odom_I.q, tf_odom_I.v);

          // update LIO result
          if (!ncrl_tf::accumTrans(LIOODOM_LAST_, LIOODOM_LAST, delta_trans))
              ROS_WARN("ACCUM TRANSFORM IS WRONG");
          ncrl_tf::setTrans(LIOODOM_LAST, LIOODOM_LAST_.q, LIOODOM_LAST_.v);

          // publish current cloud
          sensor_msgs::PointCloud2ConstPtr corner_msg = edgeLastBuf.front();
          sensor_msgs::PointCloud2ConstPtr surf_msg = flatLastBuf.front();
          pcl::PointCloud<PointType>::Ptr feature_cloud(new pcl::PointCloud<PointType>());
          pcl::PointCloud<PointType>::Ptr corner_cloud(new pcl::PointCloud<PointType>());
          pcl::PointCloud<PointType>::Ptr surf_cloud(new pcl::PointCloud<PointType>());
          pcl::fromROSMsg(*corner_msg, *corner_cloud);
          pcl::fromROSMsg(*surf_msg, *surf_cloud);
          *feature_cloud += *corner_cloud;
          *feature_cloud += *surf_cloud;

          for (int i = 0; i < (int)feature_cloud->points.size(); i++) {
              PointType pi = feature_cloud->points[i];
              Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
              ncrl_tf::Point p_;
              ncrl_tf::setPoint(p_, point_curr);
              ncrl_tf::setPointFrame(p_, "LIDAR");
              if(!ncrl_tf::TransPoint(EXTRINSIC, p_))
                ROS_WARN("trans plane point to imu frame");
              Eigen::Vector3d point_w = EST_TRANS.q * p_.point + EST_TRANS.v;
              feature_cloud->points[i].x = point_w.x();
              feature_cloud->points[i].y = point_w.y();
              feature_cloud->points[i].z = point_w.z();
          }
          sensor_msgs::PointCloud2 static_cloud;
          pcl::toROSMsg(*feature_cloud, static_cloud);
          static_cloud.header = edgeLastBuf.front()->header;
          static_cloud.header.frame_id = "WORLD";
          pub_static.publish(static_cloud);

          edgeLastBuf.pop();
          flatLastBuf.pop();
          laserOdomBuf.pop();
          fullPointsBuf.pop();
          drop_cnt++;
          continue;
      }

      // extract feature point;
      sensor_msgs::PointCloud2ConstPtr corner_msg = edgeLastBuf.front();
      sensor_msgs::PointCloud2ConstPtr surf_msg = flatLastBuf.front();
      nav_msgs::OdometryConstPtr odom_msg = laserOdomBuf.front();
      sensor_msgs::PointCloud2ConstPtr fullpoint_msg = fullPointsBuf.front();
      edgeLastBuf.pop();
      flatLastBuf.pop();
      laserOdomBuf.pop();
      fullPointsBuf.pop();

      // construct imu vector
      std::vector<sensor_msgs::ImuConstPtr> IMUs;
      while((imu_buf.front()->header.stamp.toSec() < corner_msg->header.stamp.toSec() + TD) &&
            (imu_buf.front()->header.stamp.toSec() < surf_msg->header.stamp.toSec() + TD)){
          IMUs.emplace_back(imu_buf.front());
          imu_buf.pop();
      }
      IMUs.emplace_back(imu_buf.front());
      if(IMUs.empty())
        ROS_WARN("no imu between two lidar frame");

      FeatureType feature_all(corner_msg, surf_msg);
      measurements.emplace_back(std::make_pair(IMUs, std::make_pair(odom_msg, std::make_pair(fullpoint_msg, feature_all))));

      // clear data buffer if contain more than one measurements
      while(!edgeLastBuf.empty() && !flatLastBuf.empty() &&
            !laserOdomBuf.empty() && !fullPointsBuf.empty() && measurements.size() > 0){
            double timeEdge_ = edgeLastBuf.front()->header.stamp.toSec();
            double timeFlat_ = flatLastBuf.front()->header.stamp.toSec();
            double timeOdom_ = laserOdomBuf.front()->header.stamp.toSec();
            double timeFull_ = fullPointsBuf.front()->header.stamp.toSec();
            ROS_WARN("DROP LIDAR IN ESTIMATION FOR REAL TIME PERFORMANCE");
            if (timeEdge_ == timeFlat_ && timeFlat_ == timeOdom_ && timeOdom_ == timeFull_) {
                drop_cnt++;
                edgeLastBuf.pop();
                flatLastBuf.pop();
                laserOdomBuf.pop();
                fullPointsBuf.pop();
            } else {
              break;
            }
      }
      ROS_WARN("DROP LIDAR COUNT : %d", drop_cnt);
    }
    return measurements;
}

// PUBLISH ODOMETRY AND PATH OF IMU FREQUENCY
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q,
                       const Eigen::Vector3d &V, sensor_msgs::ImuConstPtr imu_msg){
    Eigen::Quaterniond quadrotor_Q = Q ;

    // publish state on ground truth frame
    ncrl_tf::Trans trans1_, trans2_, trans3_;
    ncrl_tf::setTransFrame(trans1_, "WORLD", "IMU");
    ncrl_tf::setTransFrame(trans2_, "IMU_INIT", "IMU");
    ncrl_tf::setTransFrame(trans3_, "GT_INIT", "GT");
    ncrl_tf::setTrans(trans1_, Q, P);
    if (!ncrl_tf::deltaTrans(trans2_, WORLD2IMU, trans1_))
        ROS_WARN("TRANS WORLD TO IMU -> IMU_INIT TO IMU IN PUBLATEST ODOMETRY IS FAIL");
    if (!ncrl_tf::TransOdometry(IMU2GT, trans2_, trans3_))
        ROS_WARN("TRANS IMU INIT TO IMU -> GT INIT TO GT IN PUBLATEST ODOMETRY IS FAIL");

    nav_msgs::Odometry odometry, odometry_est;
    odometry.header = imu_msg->header;
    odometry.header.frame_id = "WORLD";
    odometry.child_frame_id = "IMU";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    odometry.twist.twist.angular.x = imu_msg->angular_velocity.x - tmp_Bg[0];
    odometry.twist.twist.angular.y = imu_msg->angular_velocity.y - tmp_Bg[1];
    odometry.twist.twist.angular.z = imu_msg->angular_velocity.z - tmp_Bg[2];

    odometry_est.header = imu_msg->header;
    odometry_est.header.frame_id = "GT_INIT";
    odometry_est.child_frame_id = "GT";
    odometry_est.pose.pose.position.x = trans3_.v.x();
    odometry_est.pose.pose.position.y = trans3_.v.y();
    odometry_est.pose.pose.position.z = trans3_.v.z();
    odometry_est.pose.pose.orientation.x = trans3_.q.x();
    odometry_est.pose.pose.orientation.y = trans3_.q.y();
    odometry_est.pose.pose.orientation.z = trans3_.q.z();
    odometry_est.pose.pose.orientation.w = trans3_.q.w();

    geometry_msgs::PoseStamped high_freq_path_msg;
    high_freq_path_msg.header = imu_msg->header;
    high_freq_path_msg.header.frame_id = "WORLD";
    high_freq_path_msg.pose.position.x = P.x();
    high_freq_path_msg.pose.position.y = P.y();
    high_freq_path_msg.pose.position.z = P.z();
    high_freq_path_msg.pose.orientation.x = Q.x();
    high_freq_path_msg.pose.orientation.y = Q.y();
    high_freq_path_msg.pose.orientation.z = Q.z();
    high_freq_path_msg.pose.orientation.w = Q.w();
    high_freq_path.header = imu_msg->header;
    high_freq_path.header.frame_id = "WORLD";
    high_freq_path.poses.push_back(high_freq_path_msg);

    // maintain size of imu propogation
    if (high_freq_path.poses.size() >= 3600)
        high_freq_path.poses.erase(high_freq_path.poses.begin());

    pub_high_freq_path.publish(high_freq_path);
    pub_latest_odometry.publish(odometry); // imu propagate mainly localize UAV
    pub_latest_odometry_est.publish(odometry_est);
}

void pubMap(std_msgs::Header &header){
    //publish map in 10 Hz
    if (lidar_id % 1 == 0){
        sensor_msgs::PointCloud2 map;
        pcl::toROSMsg(*estimator.Feature_All->laserCloudSurround, map);
        map.header = header;
        map.header.frame_id = "WORLD";
        pub_map.publish(map);
    }
    sensor_msgs::PointCloud2 scan;
    pcl::toROSMsg(*estimator.Feature_All->laserCloudRegisted, scan);
    scan.header = header;
    scan.header.frame_id = "WORLD";
    pub_scan.publish(scan);

    sensor_msgs::PointCloud2 static_cloud;
    pcl::toROSMsg(*estimator.Feature_All->laserCloudStatic, static_cloud);
    static_cloud.header = header;
    static_cloud.header.frame_id = "WORLD";
    pub_static.publish(static_cloud);
}


double computeSum = 0;
int computeCount = 0;
//  through imu_cb we can get predict term and imu_propagate odometry
void imu_cb(const sensor_msgs::ImuConstPtr &imu_msg){
    TicToc preIntegration_;
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_FATAL("imu messages in disorder!");
        return;
    }
//    if (imu_msg->header.stamp.toSec() - last_imu_t >= 0.02)
//        ROS_FATAL("imu message delay");

    last_imu_t = imu_msg->header.stamp.toSec();
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();

    if (estimator.solver_flag == NON_LINEAR){
        // update tmp_[state] for publish imu propogate
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        pubLatestOdometry(tmp_P, tmp_Q, tmp_V, imu_msg);
    }
    computeSum+=preIntegration_.toc();
    computeCount++;
}

void EdgeLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    m_buf.lock();
    edgeLastBuf.push(msg);
    m_buf.unlock();
}

void FlatLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    m_buf.lock();
    flatLastBuf.push(msg);
    m_buf.unlock();
}

void LaserOdomHandler(const nav_msgs::OdometryConstPtr &msg){
    m_buf.lock();
    laserOdomBuf.push(msg);
    m_buf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    m_buf.lock();
    fullPointsBuf.push(msg);
    m_buf.unlock();
}

void broad_tf(std_msgs::Header &header){
    static tf::TransformBroadcaster br;
    auto now = header.stamp.now();
    std::map<int, FeaturePerSweep>::reverse_iterator frame_end;
    frame_end = estimator.Feature_All->Feature_All_Map.rbegin();
    tf::Transform tf_odom_L, tf_odom_I, tf_extrinsic, tf_world, tf_curr_L, tf_curr_I, tf_curr_odom_I, tf_curr_odom_L;

    // world and imu_init
    ncrl_tf::setTfTrans(tf_world, WORLD2IMU.q, WORLD2IMU.v);
    br.sendTransform(tf::StampedTransform(tf_world.inverse(), now, WORLD2IMU.end_frame, WORLD2IMU.start_frame));

    // lidar and imu extrinsic
    ncrl_tf::setTfTrans(tf_extrinsic, EXTRINSIC.q, EXTRINSIC.v);
    br.sendTransform(tf::StampedTransform(tf_extrinsic.inverse(), now, EXTRINSIC.end_frame, EXTRINSIC.start_frame));

    // [sensor]_odom -> [sensor]
    ncrl_tf::setTfTrans(tf_odom_I, frame_end->second.Tinit2odom_I.q, frame_end->second.Tinit2odom_I.v);
    ncrl_tf::setTfTrans(tf_odom_L, frame_end->second.Tinit2odom_L.q, frame_end->second.Tinit2odom_L.v);
    br.sendTransform(tf::StampedTransform(tf_odom_I.inverse(), now, frame_end->second.Tinit2odom_I.end_frame,
                                                                    frame_end->second.Tinit2odom_I.start_frame));
    br.sendTransform(tf::StampedTransform(tf_odom_L.inverse(), now, frame_end->second.Tinit2odom_L.end_frame,
                                                                    frame_end->second.Tinit2odom_L.start_frame));
    // [sensor]_init -> [sensor]
    ncrl_tf::setTfTrans(tf_curr_L, frame_end->second.Tw2curr_L.q, frame_end->second.Tw2curr_L.v);
    ncrl_tf::setTfTrans(tf_curr_I, frame_end->second.Tw2curr_I.q, frame_end->second.Tw2curr_I.v);
    br.sendTransform(tf::StampedTransform(tf_curr_L, now, frame_end->second.Tw2curr_L.start_frame, frame_end->second.Tw2curr_L.end_frame));
    br.sendTransform(tf::StampedTransform(tf_curr_I.inverse(), now, frame_end->second.Tw2curr_I.end_frame, frame_end->second.Tw2curr_I.start_frame));

    // [sensor]_init -> [sensor]_odom_start
    ncrl_tf::setTfTrans(tf_curr_odom_I, frame_end->second.Tcurr2odom_I.q, frame_end->second.Tcurr2odom_I.v);
    ncrl_tf::setTfTrans(tf_curr_odom_L, frame_end->second.Tcurr2odom_L.q, frame_end->second.Tcurr2odom_L.v);
    br.sendTransform(tf::StampedTransform(tf_curr_odom_I, now, frame_end->second.Tcurr2odom_I.start_frame, frame_end->second.Tcurr2odom_I.end_frame));
    br.sendTransform(tf::StampedTransform(tf_curr_odom_L, now, frame_end->second.Tcurr2odom_L.start_frame, frame_end->second.Tcurr2odom_L.end_frame));

    //transform aloam to WORLD
//    br.sendTransform(tf::StampedTransform(tf_extrinsic.inverse(), now, "/LIDAR_INIT_ALOAM", "/WORLD"));
}

void process(){
    while(ros::ok()){
        // using m_buf to ensure the queue are used with on e thread
        m_buf.lock();
        CombinedData measurements;
        measurements = getMeasurements();
        m_buf.unlock();

        m_estimator.lock();
        if (measurements.size() > 0){
            TicToc t_whole;
            // first part pre-integration
            int lidar_cnt = 0;

            if (measurements.size() != 1)
                ROS_FATAL("MEASUREMENT SIZE IS LARGER THAN 1, IT MAY CAUSE DELAY PROBLEM");

            for (auto &measurement : measurements){
                lidar_id++; // 0 1 2 3 ...
                std::cout << "\n=== lidar id : " + std::to_string(lidar_id) + " ===" << std::endl;
                lidar_cnt++;
                // first part : handle imu msg and accumulate delta p, q, v on imu_i frame
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0,  rz = 0;
                for (auto &imu_msg : measurement.first)
                {
                    double t = imu_msg->header.stamp.toSec();
                    double edge_msg_t = measurement.second.second.second.first->header.stamp.toSec() + TD;
                    double flat_msg_t = measurement.second.second.second.second->header.stamp.toSec() + TD;

                    // handle imu vector with processIMU
                    if (t <= edge_msg_t && t <= flat_msg_t){
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time; // approximately 0, 1/imu_frequency
                        current_time = t; // update current time with imu time
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator.processImu(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    } else {
                        // interpolation
                        double dt_1 = edge_msg_t - current_time;
                        double dt_2 = t - edge_msg_t;
                        current_time = edge_msg_t;

                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);

                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processImu(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    }
                }
                // second part : process Lidar, define state
                FeaturePerSweep Feature_Msg;
                pcl::fromROSMsg(*measurement.second.second.second.first, *Feature_Msg.laserCloudCornerLast);
                pcl::fromROSMsg(*measurement.second.second.second.second, *Feature_Msg.laserCloudSurfLast);
                pcl::fromROSMsg(*measurement.second.second.first, *Feature_Msg.laserCloudFullRes);
                Feature_Msg.laserOdom = *measurement.second.first;
                lidar_header = Feature_Msg.laserOdom.header;

                // set transform on IMU/LIDAR frame from laser odometry
                Eigen::Quaterniond q_(Feature_Msg.laserOdom.pose.pose.orientation.w,
                                      Feature_Msg.laserOdom.pose.pose.orientation.x,
                                      Feature_Msg.laserOdom.pose.pose.orientation.y,
                                      Feature_Msg.laserOdom.pose.pose.orientation.z);
                Eigen::Vector3d v_(Feature_Msg.laserOdom.pose.pose.position.x,
                                   Feature_Msg.laserOdom.pose.pose.position.y,
                                   Feature_Msg.laserOdom.pose.pose.position.z);
                ncrl_tf::setTrans(Feature_Msg.Tinit2odom_L, q_, v_);
                // trans f2f odom from lidar_odom 2 lidar -> imu_odom 2 imu
                if(!ncrl_tf::TransOdometry(EXTRINSIC, Feature_Msg.Tinit2odom_L,
                                                      Feature_Msg.Tinit2odom_I))
                    ROS_WARN("Trans odom from lidar frame to imu frame fail");

                estimator.processLIO(Feature_Msg, lidar_id);

                // publisher
                if (estimator.solver_flag == NON_LINEAR){
                    m_state.lock();
                    update(lidar_header);

                    ncrl_tf::Trans tf_odom, delta_trans;
                    ncrl_tf::setTrans(tf_odom, estimator.Qs[SWEEP_SIZE], estimator.Ps[SWEEP_SIZE]);
                    ncrl_tf::setTransFrame(tf_odom, "IMU_INIT", "IMU");
                    ncrl_tf::setTransFrame(delta_trans, "IMU_LAST", "IMU");
                    if (!ncrl_tf::deltaTrans(delta_trans, LIOODOM_LAST, tf_odom))
                        ROS_WARN("DELTA TRANSFORM IS WRONG");
                    pubEstResult(lidar_header,
                                 delta_trans);

                    // update
                    ncrl_tf::setTrans(LIOODOM_LAST, estimator.Qs[SWEEP_SIZE],
                                                    estimator.Ps[SWEEP_SIZE]);
                    m_state.unlock();
                    pubMap(lidar_header);
                    broad_tf(lidar_header);
                } else {
                  // formulate f2f odometry from last to current
                  ncrl_tf::Trans tf_odom, delta_trans;
                  ncrl_tf::setTrans(tf_odom, Feature_Msg.Tinit2odom_I.q, Feature_Msg.Tinit2odom_I.v);
                  ncrl_tf::setTransFrame(tf_odom, "IMU_INIT", "IMU");
                  ncrl_tf::setTransFrame(delta_trans, "IMU_LAST", "IMU");
                  if (!ncrl_tf::deltaTrans(delta_trans, F2FODOM_LAST, tf_odom))
                      ROS_WARN("DELTA TRANSFORM IS WRONG");
                  pubEstResult(lidar_header,
                               delta_trans);
                }
                ncrl_tf::setTrans(F2FODOM_LAST, Feature_Msg.Tinit2odom_I.q,
                                                Feature_Msg.Tinit2odom_I.v);
            }// for (auto &measurement : measurements)
            estimator.t_whole = t_whole.toc();

            if (SHOWTIME)
              estimator.showTimeCost();
            ROS_DEBUG("IMU preintegration cost : %lf ms", computeSum / computeCount);

        }// if (measurements.size() >= 1)
        m_estimator.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ncrl_lio_estimator_node_fusion");
    ros::NodeHandle nh("~");

    // setting ros console priority "DEBUG -> INFO -> WARN -> ERROR -> FATAL"
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(nh);

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>
                              (IMU_TOPIC, 400, imu_cb, ros::TransportHints().tcpNoDelay());

    // subscribe each feature point
    ros::Subscriber subLaserCloudEdgeLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laserOdometry/laser_cloud_corner_last", 100, EdgeLastHandler);
    ros::Subscriber subLaserCloudFlatLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laserOdometry/laser_cloud_surf_last", 100, FlatLastHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
                                       ("/laserOdometry/laser_odom_to_init", 100, LaserOdomHandler);
    // subscribe full point cloud
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
                                           ("/laserOdometry/velodyne_cloud_3", 100, laserCloudFullResHandler);

    pub_odom = nh.advertise<nav_msgs::Odometry>("/estimator/laser_odom", 100);
    pub_odom_est =  nh.advertise<nav_msgs::Odometry>("/estimator/laser_odom_est", 100);
    pub_pos_path = nh.advertise<nav_msgs::Path> ("/estimator/pos_path", 10);
    pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/estimator/mapping", 10);
    pub_scan = nh.advertise<sensor_msgs::PointCloud2> ("/estimator/scan", 10);
    pub_static = nh.advertise<sensor_msgs::PointCloud2> ("/estimator/static", 10);

    // lidar and imu state on imu frame
    pub_pos = nh.advertise<geometry_msgs::PointStamped> ("/estimator/pos_state", 10);
    pub_rot = nh.advertise<geometry_msgs::PointStamped> ("/estimator/rot_state", 10);
    pub_latest_odometry = nh.advertise<nav_msgs::Odometry> ("/estimator/imu_propagate", 2);
    pub_latest_odometry_est = nh.advertise<nav_msgs::Odometry> ("/estimator/imu_propagate_est", 2);
    pub_high_freq_path = nh.advertise<nav_msgs::Path> ("/estimator/high_freq_path", 10);

    // imu state on imu frame
    pub_gyr_bias = nh.advertise<geometry_msgs::PointStamped> ("/estimator/gyr_bias", 10);
    pub_acc_bias = nh.advertise<geometry_msgs::PointStamped> ("/estimator/acc_bias", 10);
    pub_vel = nh.advertise<geometry_msgs::PointStamped> ("/estimator/vel_state", 10);

    // extrinsic state
    pub_ex = nh.advertise<geometry_msgs::PoseStamped> ("/estimator/extrinsic", 10);
    pub_T_ex = nh.advertise<geometry_msgs::PointStamped> ("/estimator/EX_trans", 10);
    pub_R_ex = nh.advertise<geometry_msgs::PointStamped> ("/estimator/EX_rot", 10);

    ncrl_tf::setTransFrame(EXTRINSIC, "IMU", "LIDAR");
    ncrl_tf::setTransFrame(WORLD2IMU, "WORLD", "IMU_INIT");
    ncrl_tf::setTransFrame(IMU2GT, "IMU", "GT");
    ncrl_tf::setTransFrame(F2FODOM_LAST, "IMU_INIT", "IMU_LAST");
    ncrl_tf::setTransFrame(LIOODOM_LAST, "IMU_INIT", "IMU_LAST");
    ncrl_tf::setTransFrame(EST_TRANS, "IMU_INIT", "IMU_LAST");

    Eigen::Quaterniond q_i(1, 0, 0, 0);
    Eigen::Vector3d v_i(0, 0, 0);

    ncrl_tf::setTrans(F2FODOM_LAST, q_i, v_i);
    ncrl_tf::setTrans(LIOODOM_LAST, q_i, v_i);
    ncrl_tf::setTrans(EST_TRANS, q_i, v_i);
    // extrinsic from lidar to imu
    Eigen::Quaterniond EQ_I2L = ncrl_tf::Euler2Q(ER_I2L);
    ncrl_tf::setTrans(EXTRINSIC, EQ_I2L, ET_I2L);
    estimator.til[0] = EXTRINSIC.v;
    estimator.qil[0] = EXTRINSIC.q;

    // intitial setting of world to imu
    ncrl_tf::setTrans(WORLD2IMU, q_i, v_i);
    // extrinsic from IMU to Gt
    Eigen::Quaterniond EQ_I2G = ncrl_tf::Euler2Q(ER_I2G);
    ncrl_tf::setTrans(IMU2GT, EQ_I2G, ET_I2G);

    // Initial INFO
    ROS_INFO("FUSION NODE WITH SWEEP SIZE %d ", SWEEP_SIZE);
    ROS_INFO("DOINH ON WORLD FRAME");
    ROS_INFO("INITIAL STRUCTURE SIZE %d", INIT_SIZE);
    ROS_INFO("FUSE WITH IMU WITH TD %f", TD);
    ROS_INFO_STREAM("extrinsic rotation matrix : \n" << EXTRINSIC.q.toRotationMatrix());
    ROS_INFO_STREAM("extrinsic translation matrix \n: " << EXTRINSIC.v.transpose());
    ROS_INFO_STREAM("extrinsic GT rotation matrix : \n" << IMU2GT.q.toRotationMatrix());
    ROS_INFO_STREAM("extrinsic GT translation matrix \n: " << IMU2GT.v.transpose());
    ROS_INFO_STREAM("lidar res " << lineRes << " " << planeRes << ", BackgroundDis " << BackgroundDis);
    ROS_INFO_STREAM(" weight of \nPLANE(F2M) / CORNER(F2M) / PLANE(F2F) / EDGE(F2F)\n     "
                    << w_f2m_flat << "     /     " << w_f2m_corner << "     /     "
                    << w_f2f_flat<< "     /     " << w_f2f_corner);
    if (ENABLE_PRIOR_FACTOR)
        ROS_INFO_STREAM("weight of EXTRINSIC \nTRANSLATION / ROTATION\n     "
                        << w_ext_tran << "     /     " << w_ext_rot);

    ROS_INFO_STREAM("\n    ENABLE_F2M_PLANE: " << ENABLE_F2M_PLANE <<
                    "\n    ENABLE_F2M_EDGE: " << ENABLE_F2M_EDGE <<
                    "\n    ENABLE_F2F_PLANE: " << ENABLE_F2F_PLANE <<
                    "\n    ENABLE_MARGINALIZATION: " << ENABLE_MARGINALIZATION <<
                    "\n    ENABLE_OPT_EXTRINSIC: " << ENABLE_OPT_EXTRINSIC <<
                    "\n    ENABLE_PRIOR_FACTOR: " << ENABLE_PRIOR_FACTOR <<
                    "\n    ENABLE_HIGHSPEED_INITIALIZATION : " << ENABLE_HIGHSPEED_INITIALIZATION <<
                    "\n    ENABLE_REMOVE_BACKGROUND : " << ENABLE_REMOVE_BACKGROUND);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
