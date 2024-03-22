#include <pcmap_fuser/pcmap_fuser.h>

#include <iostream>
#include <opencv2/highgui.hpp>

PCMapFuser::PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  m_nh = nh;
  m_pnh = pnh;
  m_init_pose_sub =
      m_nh.subscribe("/initialpose", 1, &PCMapFuser::initPoseCallback, this);

  m_target_map_cloud_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/target_map_cloud", 1, true);
  m_source_map_cloud_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/source_map_cloud", 1, true);

  m_final_cloud_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/final_cloud", 1, true);

  m_tf_timer =
      m_nh.createTimer(ros::Duration(0.1), &PCMapFuser::tfTimerCallback, this);

  m_source_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_target_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_filtered_source_map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_transformed.reset(new pcl::PointCloud<pcl::PointXYZ>);
  m_final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // set initial values
  m_fixed_floating_transform.transform.translation.x = 1;
  m_fixed_floating_transform.transform.rotation.w = 1;
  m_fixed_floating_transform.header.frame_id = "target_map";
  m_fixed_floating_transform.child_frame_id = "source_map";

  // target cloud is the fixed map.

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/ro/Documents/rooms/decat_top.pcd", *m_target_map_cloud) ==
      -1) {
    ROS_ERROR("Couldn't read file room_scan1.pcd \n");
  }
  //  source cloud is the cloud to be transformed.
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/ro/Documents/rooms/decat_bottom.pcd", *m_source_map_cloud) ==
      -1) {
    ROS_ERROR("Couldn't read file room_scan2.pcd \n");
  }

  m_orb_extractor = cv::ORB::create();

  std::pair<cv::Mat, Eigen::Vector2i> target_map_2d =
      map_closures::GenerateDensityMap(*m_target_map_cloud, 0.1, 0.05);

  std::pair<cv::Mat, Eigen::Vector2i> source_map_2d =
      map_closures::GenerateDensityMap(*m_source_map_cloud, 0.1, 0.05);

  // use opencv functions to display out map
  cv::Mat target_map_image;
  cv::normalize(target_map_2d.first, target_map_image, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
  cv::namedWindow("target_map", cv::WINDOW_NORMAL);
  cv::resizeWindow("target_map", 800, 600);
  cv::imshow("target_map", target_map_image);

  cv::Mat source_map_image;
  cv::normalize(source_map_2d.first, source_map_image, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
  cv::namedWindow("source_map", cv::WINDOW_NORMAL);
  cv::resizeWindow("source_map", 800, 600);
  cv::imshow("source_map", source_map_image);

  // compute keypoints and descriptors
  std::vector<cv::KeyPoint> target_keypoints;
  cv::Mat target_descriptors;
  m_orb_extractor->detectAndCompute(target_map_image, cv::Mat(),
                                    target_keypoints, target_descriptors);

  std::vector<cv::KeyPoint> source_keypoints;
  cv::Mat source_descriptors;
  m_orb_extractor->detectAndCompute(source_map_image, cv::Mat(),
                                    source_keypoints, source_descriptors);

  // match descriptors
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(target_descriptors, source_descriptors, matches);
  std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b) {
    return a.distance < b.distance;
  });

  // only keep the best 10 matches
  matches.erase(matches.begin() + 10, matches.end());
  // draw the best 10 matches
  cv::Mat img_matches;
  cv::drawMatches(target_map_image, target_keypoints, source_map_image,
                  source_keypoints, matches, img_matches);
  cv::namedWindow("matches", cv::WINDOW_NORMAL);
  cv::resizeWindow("matches", 800, 600);
  cv::imshow("matches", img_matches);

  cv::waitKey(0);

  pcl::toROSMsg(*m_source_map_cloud, m_source_map_cloud_ros);
  pcl::toROSMsg(*m_target_map_cloud, m_target_map_cloud_ros);
  m_source_map_cloud_ros.header.frame_id = "source_map";
  m_target_map_cloud_ros.header.frame_id = "target_map";

  m_approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  m_approximate_voxel_filter.setInputCloud(m_source_map_cloud);
  m_approximate_voxel_filter.filter(*m_filtered_source_map_cloud);

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

  m_gicp.setResolution(0.2);
  m_gicp.setNumThreads(4);
  m_gicp.setInputSource(m_source_map_cloud);
  m_gicp.setInputTarget(m_target_map_cloud);
  //   // set search method
  m_gicp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
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

  geometry_msgs::Transform transform;
  transform.translation.x = msg->pose.pose.position.x;
  transform.translation.y = msg->pose.pose.position.y;
  transform.translation.z = msg->pose.pose.position.z;
  transform.rotation = msg->pose.pose.orientation;

  Eigen::Matrix4f initial_estimate =
      tf2::transformToEigen(transform).matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  m_ndt.align(*dummy_cloud, initial_estimate);

  std::cout << "Normal Distributions Transform has converged: "
            << m_ndt.hasConverged() << " score: " << m_ndt.getFitnessScore()
            << std::endl;

  m_gicp.align(*dummy_cloud, m_ndt.getFinalTransformation());
  Eigen::Affine3d final_transform;
  final_transform.matrix() = m_gicp.getFinalTransformation().cast<double>();
  m_fixed_floating_transform.transform =
      tf2::eigenToTransform(final_transform).transform;

  std::cout << "GICP has converged: " << m_gicp.hasConverged()
            << " score: " << m_gicp.getFitnessScore() << std::endl;

  pcl::transformPointCloud(*m_source_map_cloud, *m_transformed,
                           m_gicp.getFinalTransformation());

  *m_final_cloud = *m_target_map_cloud + *m_transformed;
  pcl::toROSMsg(*m_final_cloud, m_final_cloud_ros);
  m_final_cloud_ros.header.frame_id = "target_map";
  m_final_cloud_pub.publish(m_final_cloud_ros);
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