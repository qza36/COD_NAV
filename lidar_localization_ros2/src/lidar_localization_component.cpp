#include <lidar_localization/lidar_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "chassis");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("translation_threshold", 3.0);
  declare_parameter("rotation_threshold", 0.3);
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("translation_threshold", translation_threshold_);
  get_parameter("rotation_threshold", rotation_threshold_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(translation_threshold_);
    gicp->setRotationEpsilon(rotation_threshold_);
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");

  // 检查初始位姿的坐标系是否与全局坐标系一致
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }

  // 标记初始位姿接收到
  initialpose_recieved_ = true;
  last_tf_available_ = false;
  // 保存当前位姿并发布
  corrent_pose_with_cov_stamped_ptr_ = msg;
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  // 发布 map -> odom 的 TF
  geometry_msgs::msg::TransformStamped map_to_odom_transform;
  map_to_odom_transform.header.stamp = this->get_clock()->now();
  map_to_odom_transform.header.frame_id = global_frame_id_;
  map_to_odom_transform.child_frame_id = odom_frame_id_;

  // 提取初始位姿的平移和旋转
  map_to_odom_transform.transform.translation.x = msg->pose.pose.position.x;
  map_to_odom_transform.transform.translation.y = msg->pose.pose.position.y;
  map_to_odom_transform.transform.translation.z = msg->pose.pose.position.z;
  map_to_odom_transform.transform.rotation = msg->pose.pose.orientation;

  // 通过广播器发布 TF
  broadcaster_.sendTransform(map_to_odom_transform);

  RCLCPP_INFO(get_logger(), "Published map -> odom transform");

  // 处理最后一帧点云（如果存在）
  if (last_scan_ptr_ != nullptr) {
    cloudReceived(last_scan_ptr_);
  }

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    if (!use_odom_) {
        return;
    }
    RCLCPP_INFO(get_logger(), "Odometry message received.");

    odom_velocity_ = msg;

    // 获取当前时间和时间间隔
    double current_odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt_odom = current_odom_received_time - last_odom_received_time_;
    last_odom_received_time_ = current_odom_received_time;

    // 检查时间间隔的有效性
    if (dt_odom > 1.0) {
        RCLCPP_WARN(this->get_logger(), "Odom time interval is too large: %.2f s", dt_odom);
        return;
    }
    if (dt_odom < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Odom time interval is negative: %.2f s", dt_odom);
        return;
    }

    // 校验传感器数据是否有效
    if (!msg->pose.pose.orientation.x || !msg->pose.pose.orientation.y ||
        !msg->pose.pose.orientation.z || !msg->pose.pose.orientation.w) {
        RCLCPP_ERROR(this->get_logger(), "Invalid orientation data in Odometry message.");
        return;
    }

    // 获取当前机器人位姿（四元数）
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    // 定义转换后的位姿变量
    geometry_msgs::msg::PoseStamped map_pose;
    try {
        // 从 odom 坐标系转换到 map 坐标系
        tfbuffer_.transform(odom_pose, map_pose, global_frame_id_);  
        // 更新位姿信息（global_frame_id坐标系下的位姿）
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position = map_pose.pose.position;
        corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = map_pose.pose.orientation;

        // 标准化四元数（防止漂移）
        tf2::Quaternion new_quat;
        tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, new_quat);
        new_quat.normalize();  // 标准化四元数，避免误差积累

        // 更新机器人的位置和姿态
        corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = tf2::toMsg(new_quat);

        // 调试信息，记录更新的位姿
        if (enable_debug_) {
            RCLCPP_INFO(this->get_logger(), "Updated position in map frame: [%.2f, %.2f, %.2f], Orientation: [%.2f, %.2f, %.2f, %.2f]",
                        map_pose.pose.position.x,
                        map_pose.pose.position.y,
                        map_pose.pose.position.z,
                        map_pose.pose.orientation.w,
                        map_pose.pose.orientation.x,
                        map_pose.pose.orientation.y,
                        map_pose.pose.orientation.z);
        }

    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform odom to map: %s", e.what());
        return;
    }
}



void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {
    return;  // 如果没有启用 IMU 数据，直接返回
  }

  // 定义用于存储转换后的 IMU 数据
  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    // 查找从 IMU 坐标系到目标坐标系的转换矩阵
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
      base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    // 进行坐标转换：角速度、线加速度和四元数
    tf_converted_imu.angular_velocity = transformVector3(msg->angular_velocity, transform);
    tf_converted_imu.linear_acceleration = transformVector3(msg->linear_acceleration, transform);
    tf_converted_imu.orientation = transformQuaternion(msg->orientation, transform);
  } 
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    return;
  }

  // 将转换后的 IMU 数据传递给 lidar_undistortion 进行后续处理
  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, 
                                tf_converted_imu.angular_velocity.y, 
                                tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, 
                      tf_converted_imu.linear_acceleration.y, 
                      tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{tf_converted_imu.orientation.w, 
                           tf_converted_imu.orientation.x, 
                           tf_converted_imu.orientation.y, 
                           tf_converted_imu.orientation.z};

  double imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // 传递 IMU 数据到 lidar_undistortion 进行去畸变处理
  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

// 坐标转换：对 Vector3 类型的数据进行坐标转换
geometry_msgs::msg::Vector3 transformVector3(const geometry_msgs::msg::Vector3 &vec,
                                             const geometry_msgs::msg::TransformStamped &transform)
{
    // 创建一个 Vector3Stamped 对象，将输入的 Vector3 封装在其中
    geometry_msgs::msg::Vector3Stamped vec_stamped;
    // 由于 Vector3 没有 header，无法直接访问。需要手动设置它的 header。
    vec_stamped.header.frame_id = transform.header.frame_id;  // 使用变换的 frame_id 作为输出的 frame_id
    vec_stamped.header.stamp = transform.header.stamp;  // 使用变换的时间戳
    vec_stamped.vector = vec;  // 设置输入的 Vector3

    // 创建一个用于存放变换结果的 Vector3Stamped 对象
    geometry_msgs::msg::Vector3Stamped transformed_vec;

    // 执行坐标变换
    try {
        tf2::doTransform(vec_stamped, transformed_vec, transform);  // 执行变换
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("transformVector3"), "Error transforming vector: %s", e.what());
        throw;  // 或者根据需要返回空值或做其他处理
    }

    // 返回变换后的 Vector3
    return transformed_vec.vector;
}

// 坐标转换：对四元数进行坐标转换
geometry_msgs::msg::Quaternion transformQuaternion(const geometry_msgs::msg::Quaternion &quat,
                                                   const geometry_msgs::msg::TransformStamped &transform)
{
    // 将四元数消息转换为 tf2::Quaternion
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);

    // 将 TransformStamped 转换为 tf2::Transform 对象
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    // 对四元数进行坐标变换
    tf2::Quaternion transformed_quat = tf_transform * tf_quat;  // 使用 tf2::Transform 乘以 tf2::Quaternion

    // 将变换后的四元数转换为 ROS 消息格式并返回
    return tf2::toMsg(transformed_quat);
}

Eigen::Matrix4f PCLLocalization::getInitialGuess() {
    try {
        // 获取 map 到 odom 的变换
        geometry_msgs::msg::TransformStamped transform_stamped = tfbuffer_.lookupTransform(
            global_frame_id_, odom_frame_id_, tf2::TimePointZero);

        // 将变换转换为 Eigen 格式
        Eigen::Affine3d affine = tf2::transformToEigen(transform_stamped);

        // 返回当前变换作为初始猜测
        return affine.matrix().cast<float>();
    } catch (const tf2::TransformException &ex) {
        // 记录错误日志
        RCLCPP_ERROR(rclcpp::get_logger("initial_guess"), "Failed to get transform: %s", ex.what());

        // 特殊化处理：返回一个明显不同的矩阵（比如平移设置为负值或非单位旋转矩阵）
        Eigen::Matrix4f error_guess = Eigen::Matrix4f::Identity();
        error_guess(0, 3) = -999.0f; // 用于区分失败状态
        return error_guess;
    }
}


void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!map_recieved_ || !initialpose_recieved_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "cloudReceived");

  // 创建 PointCloud2 转换后的 pcl 点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  // IMU去畸变 (如果需要)
  if (use_imu_) {
    double received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  // 使用体素滤波器减少点云的大小
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  // 按距离筛选点云
  pcl::PointCloud<pcl::PointXYZI> tmp;
  tmp.reserve(cloud_ptr->points.size());  // 预分配内存，提升性能
  double r;
  for (const auto & p : cloud_ptr->points) {
    r = sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }

  if(tmp.size() < 30)
  {
    RCLCPP_WARN(get_logger(), "No Enough Points");
    return;
  }

  // 将过滤后的点云转换为 ROS 消息
  sensor_msgs::msg::PointCloud2 pruned_cloud_ros;
  pcl::toROSMsg(tmp, pruned_cloud_ros);

  // 将变换后的点云转换为 pcl 格式
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(pruned_cloud_ros, *transformed_cloud_ptr);

  // 设置点云源进行配准
  registration_->setInputSource(transformed_cloud_ptr);

  // 获取当前的位姿作为初始猜测
  Eigen::Matrix4f init_guess = getInitialGuess();

  if (init_guess.block<3, 1>(0, 3).x() == -999.0f) {
    RCLCPP_WARN(get_logger(), "Initial guess retrieval failed. Handling fallback.");
    // 跳过当前帧处理
    return;
  }

  // 点云配准
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();

  // 检查配准是否成功
  bool has_converged = registration_->hasConverged();
  double fitness_score = registration_->getFitnessScore();
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }
  if (fitness_score > score_threshold_) {
    RCLCPP_WARN(get_logger(), "The fitness score is over %lf.", score_threshold_);
    return;
  }

  // 获取最终变换矩阵
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  // 检查新变换与当前 TF 的差异
  if (last_tf_available_) {
    // 获取当前的初始猜测（map -> odom 的变换矩阵）
    Eigen::Matrix4f current_guess = getInitialGuess();

    // 计算初始猜测与最终变换的偏差
    Eigen::Matrix4f delta_transformation = current_guess.inverse() * final_transformation;

    // 分别计算平移和旋转的差异
    double delta_translation = delta_transformation.block<3, 1>(0, 3).norm(); // 平移差异
    double delta_rotation = acos(std::clamp(0.5 * (delta_transformation(0, 0) + delta_transformation(1, 1) +
                                                   delta_transformation(2, 2) - 1),
                                            -1.0, 1.0)); // 旋转差异
    if (enable_debug_) {
      // 打印初始猜测和偏差
      RCLCPP_INFO(get_logger(), "Initial guess: \n%s", current_guess.format(Eigen::IOFormat(4)));
      RCLCPP_INFO(get_logger(), "Delta translation: %f, Delta rotation: %f",
                  delta_translation, delta_rotation);
    }

    // 如果平移或旋转偏差超过阈值，发出警告并跳过更新
    if (delta_translation > translation_threshold_ || delta_rotation > rotation_threshold_) {
        RCLCPP_WARN(get_logger(), "Transformation deviation too large. Skipping update.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Transformation update passed with delta translation: %f, delta rotation: %f",
                delta_translation, delta_rotation);
  }

  auto current_time = this->get_clock()->now();
  // // 发布新的位姿
  // corrent_pose_with_cov_stamped_ptr_->header.stamp = current_time;
  // corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  // corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  // corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  // pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  // 发布变换
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = current_time;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = odom_frame_id_;
  transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
  transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
  transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
  transform_stamped.transform.rotation = quat_msg;
  broadcaster_.sendTransform(transform_stamped);
  last_tf_available_ = true;
  // // 路径更新
  // geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
  // pose_stamped_ptr->header.stamp = current_time;
  // pose_stamped_ptr->header.frame_id = global_frame_id_;
  // pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  // path_ptr_->poses.push_back(*pose_stamped_ptr);
  // path_pub_->publish(*path_ptr_);

  // 存储最后一帧扫描数据
  last_scan_ptr_ = msg;

  // 调试输出
  if (enable_debug_) {
    RCLCPP_INFO(get_logger(),"------------------------------------------------");
    RCLCPP_INFO(get_logger(), "Align time: %f [sec]", (time_align_end - time_align_start).seconds());
    RCLCPP_INFO(get_logger(), "Final transformation:(%f,%f,%f)", transform_stamped.transform.translation.x,transform_stamped.transform.translation.y,transform_stamped.transform.translation.z);
    
    // 计算旋转角度变化
    double init_cos_angle = 0.5 * (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 * (final_transformation.coeff(0, 0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    RCLCPP_INFO(get_logger(), "Delta angle: %f [deg]", delta_angle * 180 / M_PI);
    RCLCPP_INFO(get_logger(),"------------------------------------------------");
  }
}
