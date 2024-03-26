#include <pcmap_fuser/pcmap_fuser.h>

#include <iostream>
#include <opencv2/highgui.hpp>

PCMapFuser::PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  m_nh = nh;
  m_pnh = pnh;
  m_init_pose_sub =
      m_nh.subscribe("/initialpose", 1, &PCMapFuser::initPoseCallback, this);

  m_marker_pub =
      m_nh.advertise<visualization_msgs::MarkerArray>("/marker", 1, true);

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
          "/home/ro/Documents/rooms/decat_bottom_rotated.pcd",
          *m_source_map_cloud) == -1) {
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
  cv::resizeWindow("source_map", 922, 1267);
  cv::imshow("source_map", source_map_image);

  // compute keypoints and descriptors
  std::vector<cv::KeyPoint> target_keypoints;
  cv::Mat target_descriptors;
  m_orb_extractor->detectAndCompute(target_map_2d.first, cv::noArray(),
                                    target_keypoints, target_descriptors);

  std::vector<cv::KeyPoint> source_keypoints;
  cv::Mat source_descriptors;
  m_orb_extractor->detectAndCompute(source_map_2d.first, cv::noArray(),
                                    source_keypoints, source_descriptors);

  // match descriptors
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(target_descriptors, source_descriptors, matches);

  // compute offset from initial estimate position
  if (matches.size() >= 3) {
    auto source_map_lower_bound = source_map_2d.second;
    auto target_map_lower_bound = target_map_2d.second;

    std::vector<map_closures::PointPair> keypoint_pairs;
    keypoint_pairs.reserve(matches.size());

    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch &a, const cv::DMatch &b) {
                return a.distance < b.distance;
              });
    for (int i = 0; i < 3; i++) {
      auto curr = matches[i];
      auto target_keypoint = target_keypoints[curr.queryIdx].pt;
      auto source_keypoint = source_keypoints[curr.trainIdx].pt;
      keypoint_pairs.emplace_back(
          Eigen::Vector2d(source_keypoint.y + source_map_lower_bound.x() + 10,
                          source_keypoint.x + source_map_lower_bound.y()),
          Eigen::Vector2d(target_keypoint.y + target_map_lower_bound.x(),
                          target_keypoint.x + target_map_lower_bound.y()));
    }

    // print the source image width and height
    std::cout << "source image width: " << source_map_2d.first.cols
              << " source image height: " << source_map_2d.first.rows << "\n";

    std::cout << "target lower x: " << target_map_lower_bound.x()
              << " target lower y: " << target_map_lower_bound.y() << "\n";

    std::cout << "source lower x: " << source_map_lower_bound.x()
              << " source lower y: " << source_map_lower_bound.y() << "\n";

    // publish the lower bound coorinates as markers
    // addMarker("lower_bound", target_map_lower_bound.x()/10.0,
    // target_map_lower_bound.y()/10.0); addMarker("source_bound",
    // source_map_lower_bound.x()/10.0, source_map_lower_bound.y()/10.0);
    addMarker("ref1", 11.1, -61.62);
    addMarker("ref2", 9.42, -60.0);
    addMarker("query1", 13.9, -46.9);
    addMarker("query2", 13.6, -49.1);
    addMarker("source_lowest", -60.7, -99.3);

    // find the smallest x and y values in the m_source_map_cloud
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    for (auto &point : m_source_map_cloud->points) {
      if (point.x < min_x) {
        min_x = point.x;
      }
      if (point.y < min_y) {
        min_y = point.y;
      }
    }
    // print it out
    std::cout << "min x: " << min_x << " min y: " << min_y << std::endl;

    std::vector<cv::DMatch> print_matches;
    for (int i = 0; i < 3; i++) {
      print_matches.push_back(matches[i]);
    }

    // compute the initial translation and rotation
    Eigen::Isometry2d estimated_offset =
        map_closures::KabschUmeyamaAlignment2D(keypoint_pairs);

    // print out the estimated translation and rotation
    std::cout << "Estimated translation: " << estimated_offset.translation().x()
              << " " << estimated_offset.translation().y() << std::endl;

    std::cout << "Estimated rotation: " << estimated_offset.linear().matrix()
              << std::endl;

    // apply estimated offset from initial position
    m_fixed_floating_transform.transform.translation.x +=
        estimated_offset.translation().x() / 10.0;
    m_fixed_floating_transform.transform.translation.y +=
        estimated_offset.translation().y() / 10.0;

    // convert the rotation matrix to quaternion
    Eigen::Matrix3d rotation_matrix_3d = Eigen::Matrix3d::Identity();
    rotation_matrix_3d.block<2, 2>(0, 0) = estimated_offset.linear();
    Eigen::Quaterniond q(rotation_matrix_3d);
    // set the quaternion to be the one above
    m_fixed_floating_transform.transform.rotation.x = q.x();
    m_fixed_floating_transform.transform.rotation.y = q.y();
    m_fixed_floating_transform.transform.rotation.z = q.z();
    m_fixed_floating_transform.transform.rotation.w = q.w();

    // cv::Mat img_matches;
    // cv::drawMatches(target_map_image, target_keypoints, source_map_image,
    //                 source_keypoints, print_matches, img_matches);
    // cv::namedWindow("matches", cv::WINDOW_NORMAL);
    // cv::resizeWindow("matches", 800, 600);
    // cv::imshow("matches", img_matches);

    // cv::waitKey(0);
  } else {
    ROS_WARN("Automated initialization is not available.");
  }

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
  Eigen::Matrix4f initial_estimate =
      tf2::transformToEigen(m_fixed_floating_transform.transform)
          .matrix()
          .cast<float>();

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

  // publish to marker topic
  m_marker_pub.publish(markers);
}

void PCMapFuser::addMarker(std::string name, double x, double y) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "target_map";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  markers.markers.push_back(marker);
}