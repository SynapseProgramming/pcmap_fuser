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
}

void PCMapFuser::tfTimerCallback(const ros::TimerEvent &event) {
  std::cout << "Timer callback\n" << std::endl;
  m_fixed_floating_transform.header.stamp = ros::Time::now();
  m_broadcaster.sendTransform(m_fixed_floating_transform);

  m_source_map_cloud_ros.header.stamp = ros::Time::now();
  m_target_map_cloud_ros.header.stamp = ros::Time::now();

  // publish the pointclouds
  m_source_map_cloud_pub.publish(m_source_map_cloud_ros);
  m_target_map_cloud_pub.publish(m_target_map_cloud_ros);
}