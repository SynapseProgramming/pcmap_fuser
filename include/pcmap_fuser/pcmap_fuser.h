#ifndef PCMAP_FUSER_H
#define PCMAP_FUSER_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <pcmap_fuser/density_map.hpp>
#include <pcmap_fuser/AlignRansac2D.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class PCMapFuser {
 public:
  PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh);

 private:
  void initPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void tfTimerCallback(const ros::TimerEvent &event);


  void addMarker(std::string name, double x, double y);

  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;
  ros::Subscriber m_init_pose_sub;
  ros::Publisher m_target_map_cloud_pub;
  ros::Publisher m_source_map_cloud_pub;
  ros::Publisher m_final_cloud_pub;
  ros::Publisher m_marker_pub;
  visualization_msgs::MarkerArray markers;

  ros::Timer m_tf_timer;

  tf2_ros::TransformBroadcaster m_broadcaster;

  geometry_msgs::TransformStamped m_fixed_floating_transform;
  sensor_msgs::PointCloud2 m_source_map_cloud_ros;
  sensor_msgs::PointCloud2 m_target_map_cloud_ros;
  sensor_msgs::PointCloud2 m_final_cloud_ros;

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_transformed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_final_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_map_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_target_map_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_source_map_cloud;

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> m_approximate_voxel_filter;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> m_ndt;
  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> m_gicp;

  cv::Ptr<cv::DescriptorExtractor> m_orb_extractor;
};

#endif  // PCMAP_FUSER_H
