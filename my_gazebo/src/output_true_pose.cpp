#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

unsigned int count = 0;
ros::Publisher pub_gt;
nav_msgs::Path cur_path;
geometry_msgs::PoseStamped pose;

void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  int i = 0;
  int idx = -1;
  std::string robot("mobile_base::base_link");
  for (i = 0; i < msg->name.size(); i++) {
    if (msg->name[i].compare(robot) == 0) {
      idx = i;
      break;
    }
  }
  cur_path.header.stamp = ros::Time::now();
  cur_path.header.seq = count;
  pose.pose = msg->pose[idx];
  pose.header = cur_path.header;
  cur_path.poses.push_back(pose);
  pub_gt.publish(cur_path);
  ROS_INFO("I heard: [%f, %f, %f]", msg->pose[idx].position.x,
                                    msg->pose[idx].position.y,
                                    msg->pose[idx].position.z);
  count += 1;
}

int main(int argc, char **argv)
{
  cur_path.header.frame_id = "/lsm_odom";
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, chatterCallback);
  pub_gt = n.advertise<nav_msgs::Path>("gt_pose", 1);
  
  ros::spin();

  return 0;
}