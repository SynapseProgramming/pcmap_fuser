#ifndef PCMAP_FUSER_H
#define PCMAP_FUSER_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
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

  ros::Timer m_tf_timer;

  tf2_ros::TransformBroadcaster m_broadcaster;

  geometry_msgs::TransformStamped m_fixed_floating_transform;
};

#endif  // PCMAP_FUSER_H
