#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

class PathContainer
{
public:
  PathContainer()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param("target_frame_name", p_target_frame_name_, std::string("lsm_odom"));
    private_nh.param("source_frame_name", p_source_frame_name_, std::string("base_link"));
    private_nh.param("trajectory_update_rate", p_trajectory_update_rate_, 4.0);
    private_nh.param("trajectory_publish_rate", p_trajectory_publish_rate_, .5);

    waitForTf();
    
    ros::NodeHandle nh;
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("gtsam_pose", 1);
    odom_pub_ = nh.advertise<geometry_msgs::PoseStamped>("gtsam_odom", 1);

    update_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_update_rate_), 
                                                      &PathContainer::trajectoryUpdateTimerCallback, this, false);
    publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), 
                                                       &PathContainer::publishTrajectoryTimerCallback, this, false);

    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;

    cur_path.header.frame_id = p_target_frame_name_;
    gtsam_path.header.frame_id = p_target_frame_name_;
    
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
  
    count = -1;
  }

  bool poseDifferenceLargerThan(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
  {
    double translate_thresh = 0.05;
    double rotate_thresh = 0.05;
    
    tf::Stamped<tf::Pose> tfpose1;
    tf::poseStampedMsgToTF(p1, tfpose1);
    tf::Stamped<tf::Pose> tfpose2;
    tf::poseStampedMsgToTF(p2, tfpose2);
        
    tf::Pose diff = tfpose1.inverseTimes(tfpose2);
    double del_translate = diff.getOrigin().length();
    
    tf::Quaternion tfq(diff.getRotation().x(),
                       diff.getRotation().y(),
                       diff.getRotation().z(),
                       diff.getRotation().w());
    tf::Matrix3x3 tfm(tfq);
    double r, p, y;
    tfm.getRPY(r, p, y);
    double del_rotate = y;
    
    if ((del_translate > translate_thresh) || (del_rotate > rotate_thresh))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  void addCurrentTfPoseToTrajectory()
  {
    pose_source_.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped pose_out;

    tf_.transformPose(p_target_frame_name_, pose_source_, pose_out);

    if (cur_path.poses.size() != 0)
    {
      //Only add pose to trajectory if it's not already stored
      if (pose_out.header.stamp != cur_path.poses.back().header.stamp)
      {
        if (poseDifferenceLargerThan(cur_path.poses.back(), pose_out))
        {
          cur_path.poses.push_back(pose_out);
          gtsam_path.poses.push_back(pose_out);
          count += 1;
          add_to_gtsam();
        }
      }
    }
    else
    {
      cur_path.poses.push_back(pose_out);
      gtsam_path.poses.push_back(pose_out);
      count += 1;
      add_to_gtsam();
    }

    cur_path.header.stamp = pose_out.header.stamp;
    gtsam_path.header.stamp = pose_out.header.stamp;
  }
  
  void add_to_gtsam()
  {

      gtsam::Point3 t = gtsam::Point3(cur_path.poses.back().pose.position.x,
                                      cur_path.poses.back().pose.position.y,
                                      cur_path.poses.back().pose.position.z);
      gtsam::Rot3 R = gtsam::Rot3::quaternion(cur_path.poses.back().pose.orientation.w,
                                              cur_path.poses.back().pose.orientation.x,
                                              cur_path.poses.back().pose.orientation.y,
                                              cur_path.poses.back().pose.orientation.z);
      gtsam::Pose3 p = gtsam::Pose3(R, t);

      // Add an initial guess for the current pose
      //initialEstimate.insert(gtsam::Symbol('x', count), p);
      initialEstimate.insert(count, p);
      // If this is the first iteration, add a prior on the first pose to set the coordinate frame
      // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
      // adding it to iSAM.
      gtsam::noiseModel::Diagonal::shared_ptr poseNoise;
      gtsam::noiseModel::Diagonal::shared_ptr odomNoise;

      poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1))); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      odomNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1))); 
      
      if (count == 0) {
        // Add a prior on pose x0  
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(count, p, poseNoise));
      }
      else if (count > 0 ) {
        // Create odometry (Between) factors between consecutive poses
        
        tf::Stamped<tf::Pose> tfpose1;
        tf::poseStampedMsgToTF(cur_path.poses[count-1], tfpose1);
        tf::Stamped<tf::Pose> tfpose2;
        tf::poseStampedMsgToTF(cur_path.poses[count], tfpose2);
        
        tf::Pose tfodom = tfpose1.inverseTimes(tfpose2);

        gtsam::Point3 t = gtsam::Point3(tfodom.getOrigin().x(),
                                        tfodom.getOrigin().y(),
                                        tfodom.getOrigin().z()
                                       );
        gtsam::Rot3 R = gtsam::Rot3::quaternion(tfodom.getRotation().w(),
                                                tfodom.getRotation().x(),
                                                tfodom.getRotation().y(),
                                                tfodom.getRotation().z()
                                               );
        gtsam::Pose3 odometry = gtsam::Pose3(R, t);
          
        tf::Quaternion tfq(tfodom.getRotation().x(),
                           tfodom.getRotation().y(),
                           tfodom.getRotation().z(),
                           tfodom.getRotation().w());
        tf::Matrix3x3 tfm(tfq);
        
//         graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', count-1), gtsam::Symbol('x', count), odometry, odomNoise));
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>((count-1), (count), odometry, odomNoise));

        // Update iSAM with the new factors
        isam.update(graph, initialEstimate);
        // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
        // If accuracy is desired at the expense of time, update(*) can be called additional times
        // to perform multiple optimizer iterations every step.
        isam.update();

        gtsam::Values currentEstimate = isam.calculateBestEstimate();
        gtsam::Pose3 p;
        gtsam::Values::iterator it;
        for (it = currentEstimate.begin(); it != currentEstimate.end(); it++)
        {
          p = (gtsam::Pose3&) (*it).value;
          cur_path.poses[(*it).key].pose.position.x = p.x();
          cur_path.poses[(*it).key].pose.position.y = p.y();
          cur_path.poses[(*it).key].pose.position.z = p.z();
          gtsam_path.poses[(*it).key].pose.position.x = p.x();
          gtsam_path.poses[(*it).key].pose.position.y = p.y();
          gtsam_path.poses[(*it).key].pose.position.z = p.z();
        }
        
        // Clear the factor graph and values for the next iteration
        graph.resize(0);
        initialEstimate.clear();
      } 
  }

  void trajectoryUpdateTimerCallback(const ros::TimerEvent& event)
  {
    try
    {
      addCurrentTfPoseToTrajectory();
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Trajectory Server: Transform from %s to %s failed: %s \n", 
               p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what() );
    }
  }
  
  void publishTrajectoryTimerCallback(const ros::TimerEvent& event)
  {
    trajectory_pub_.publish(gtsam_path);
  }
  
  void waitForTf()
  {
    ros::WallTime start = ros::WallTime::now();
    ROS_INFO("Waiting for tf transform data between frames %s and %s to become available", 
             p_target_frame_name_.c_str(), p_source_frame_name_.c_str() );

    bool transform_successful = false;

    while (!transform_successful)
    {
      transform_successful = tf_.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time());
      if (transform_successful) break;

      ros::WallTime now = ros::WallTime::now();

      if ((now-start).toSec() > 20.0)
      {
        ROS_WARN_ONCE("No transform between frames %s and %s available after %f seconds of waiting. This warning only prints once.", 
                      p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now-start).toSec());
      }
      
      if (!ros::ok()) return;
      ros::WallDuration(1.0).sleep();
    }

    ros::WallTime end = ros::WallTime::now();
    ROS_INFO("Finished waiting for tf, waited %f seconds", (end-start).toSec());
  }

   //parameters
  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  // Zero pose used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  ros::Timer update_trajectory_timer_;
  ros::Timer publish_trajectory_timer_;

  ros::Publisher trajectory_pub_;
  ros::Publisher odom_pub_;

  tf::TransformListener tf_;
  
  unsigned int count;
  nav_msgs::Path cur_path;
  nav_msgs::Path gtsam_path;
  
  // Initialize iSAM
  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;
  gtsam::ISAM2Params parameters;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gtsam_pose");

  PathContainer pc;

  ros::spin();

  return 0;
}