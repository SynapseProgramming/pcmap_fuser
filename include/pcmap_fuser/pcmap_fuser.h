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

class PCMapFuser {
 public:
  PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh);

 private:
  void initPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void tfTimerCallback(const ros::TimerEvent &event);

  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;
  ros::Subscriber m_init_pose_sub;
  ros::Publisher m_target_map_cloud_pub;
  ros::Publisher m_source_map_cloud_pub;

  ros::Timer m_tf_timer;

  tf2_ros::TransformBroadcaster m_broadcaster;

  geometry_msgs::TransformStamped m_fixed_floating_transform;
  sensor_msgs::PointCloud2 m_source_map_cloud_ros;
  sensor_msgs::PointCloud2 m_target_map_cloud_ros;

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_map_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_target_map_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_source_map_cloud;

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> m_approximate_voxel_filter;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> m_ndt;
};

#endif  // PCMAP_FUSER_H
