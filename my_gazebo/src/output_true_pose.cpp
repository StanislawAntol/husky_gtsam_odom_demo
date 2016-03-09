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
bool update_traj = false;
bool publish_traj = false;

void trajectoryUpdateTimerCallback(const ros::TimerEvent& event)
{
  update_traj = true;    
}

void publishTrajectoryTimerCallback(const ros::TimerEvent& event)
{
  publish_traj = true;    
}

void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  int i = 0;
  int idx = -1; 
  if (update_traj)
  {
    update_traj = false;
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
    pose.pose.position.z = 0.0;
    pose.header = cur_path.header;
    cur_path.poses.push_back(pose);
  }
  if (publish_traj) {
    publish_traj = false;
    pub_gt.publish(cur_path);
  }
  count += 1;
}

int main(int argc, char **argv)
{
  cur_path.header.frame_id = "/lsm_odom";
  ros::init(argc, argv, "listener");

  ros::NodeHandle private_nh("~");
  double p_trajectory_update_rate_ = 10.0;
  double p_trajectory_publish_rate_ = 2;
  
  ros::Timer update_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_update_rate_), trajectoryUpdateTimerCallback);
  ros::Timer publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), publishTrajectoryTimerCallback);
    
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000, chatterCallback);
  pub_gt = n.advertise<nav_msgs::Path>("gt_pose", 1);
  
  ros::spin();

  return 0;
}