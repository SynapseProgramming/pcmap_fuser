#include<pcmap_fuser/pcmap_fuser.h>
#include <ros/ros.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcmap_fuser_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PCMapFuser fuser(nh, pnh);

  ros::spin();
  return 0;
}   