#include <pcmap_fuser/pcmap_fuser.h>

#include <iostream>

PCMapFuser::PCMapFuser(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  m_nh = nh;
  m_pnh = pnh;
  m_init_pose_sub =
      m_nh.subscribe("/initialpose", 1, &PCMapFuser::initPoseCallback, this);

  m_tf_timer =
      m_nh.createTimer(ros::Duration(0.1), &PCMapFuser::tfTimerCallback, this);

  // set initial values
  m_fixed_floating_transform.transform.translation.x = 1;
  m_fixed_floating_transform.transform.rotation.w = 1;
  m_fixed_floating_transform.header.frame_id = "map";
  m_fixed_floating_transform.child_frame_id = "base_link";
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
}