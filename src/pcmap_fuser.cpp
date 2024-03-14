#include <pcmap_fuser/pcmap_fuser.h>

#include <iostream>

PCMapFuser::PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  m_nh = nh;
  m_pnh = pnh;
  m_init_pose_sub =
      m_nh.subscribe("/initialpose", 1, &PCMapFuser::initPoseCallback, this);

  m_target_map_cloud_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/target_map_cloud", 1, true);
  m_source_map_cloud_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/source_map_cloud", 1, true);

  m_tf_timer =
      m_nh.createTimer(ros::Duration(0.1), &PCMapFuser::tfTimerCallback, this);

  m_source_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_target_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_filtered_source_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // set initial values
  m_fixed_floating_transform.transform.translation.x = 1;
  m_fixed_floating_transform.transform.rotation.w = 1;
  m_fixed_floating_transform.header.frame_id = "target_map";
  m_fixed_floating_transform.child_frame_id = "source_map";

  // target cloud is the fixed map.

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/ro/Documents/rooms/room_scan1.pcd", *m_target_map_cloud) ==
      -1) {
    ROS_ERROR("Couldn't read file room_scan1.pcd \n");
  }
  //  source cloud is the cloud to be transformed.
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/ro/Documents/rooms/room_scan2.pcd", *m_source_map_cloud) ==
      -1) {
    ROS_ERROR("Couldn't read file room_scan2.pcd \n");
  }

  pcl::toROSMsg(*m_source_map_cloud, m_source_map_cloud_ros);
  pcl::toROSMsg(*m_target_map_cloud, m_target_map_cloud_ros);
  m_source_map_cloud_ros.header.frame_id = "source_map";
  m_target_map_cloud_ros.header.frame_id = "target_map";

  m_approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  m_approximate_voxel_filter.setInputCloud(m_source_map_cloud);
  m_approximate_voxel_filter.filter(*m_filtered_source_map_cloud);
  // print filtered cloud size
  std::cout << "Filtered cloud size: " << m_filtered_source_map_cloud->size()
            << std::endl;


  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination
  // condition.
  m_ndt.setTransformationEpsilon(0.001);
  // Setting maximum step size for More-Thuente line search.
  m_ndt.setStepSize(0.2);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  m_ndt.setResolution(1);
  // Setting max number of registration iterations.
  m_ndt.setMaximumIterations(100);
  m_ndt.setInputSource(m_filtered_source_map_cloud);
  m_ndt.setInputTarget(m_target_map_cloud);
}

void PCMapFuser::initPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  m_fixed_floating_transform.transform.translation.x =
      msg->pose.pose.position.x;
  m_fixed_floating_transform.transform.translation.y =
      msg->pose.pose.position.y;
  m_fixed_floating_transform.transform.translation.z =
      msg->pose.pose.position.z;
  m_fixed_floating_transform.transform.rotation = msg->pose.pose.orientation;

  // print out the transform values
  std::cout << "before: x: "
            << m_fixed_floating_transform.transform.translation.x
            << " y: " << m_fixed_floating_transform.transform.translation.y
            << " z: " << m_fixed_floating_transform.transform.translation.z
            << std::endl;

  geometry_msgs::Transform transform;
  transform.translation.x = msg->pose.pose.position.x;
  transform.translation.y = msg->pose.pose.position.y;
  transform.translation.z = msg->pose.pose.position.z;
  transform.rotation = msg->pose.pose.orientation;

  Eigen::Matrix4f initial_estimate =
      tf2::transformToEigen(transform).matrix().cast<float>();

  // Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  // Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  m_ndt.align(*dummy_cloud, initial_estimate);
  std::cout << "Normal Distributions Transform has converged: "
            << m_ndt.hasConverged() << " score: " << m_ndt.getFitnessScore()
            << std::endl;

  Eigen::Affine3d final_transform;
  final_transform.matrix() = m_ndt.getFinalTransformation().cast<double>();
  m_fixed_floating_transform.transform =
      tf2::eigenToTransform(final_transform).transform;

  // print out the transform values
  std::cout << "after: x: "
            << m_fixed_floating_transform.transform.translation.x
            << " y: " << m_fixed_floating_transform.transform.translation.y
            << " z: " << m_fixed_floating_transform.transform.translation.z
            << std::endl;
}

void PCMapFuser::tfTimerCallback(const ros::TimerEvent &event) {
  m_fixed_floating_transform.header.stamp = ros::Time::now();
  m_broadcaster.sendTransform(m_fixed_floating_transform);

  m_source_map_cloud_ros.header.stamp = ros::Time::now();
  m_target_map_cloud_ros.header.stamp = ros::Time::now();

  // publish the pointclouds
  m_source_map_cloud_pub.publish(m_source_map_cloud_ros);
  m_target_map_cloud_pub.publish(m_target_map_cloud_ros);
}