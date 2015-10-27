#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

ros::Publisher odom_pub;
bool received_posestamped_ = false;
geometry_msgs::PoseStamped latest_posestamped_msg_;
geometry_msgs::PoseStamped last_used_posestamped_msg_;
ros::Time new_time_;
ros::Time old_time_;
std::string parent_id_ = "odom";
std::string frame_id_ = "base_link";

void posestampedCallback(const geometry_msgs::PoseStamped::ConstPtr& posestamped_msg)
{
  if (!received_posestamped_)
  {
    last_used_posestamped_msg_ = *posestamped_msg;
    old_time_ = ros::Time::now();
    received_posestamped_ = true;
  }
  latest_posestamped_msg_ = *posestamped_msg;
  new_time_ = ros::Time::now();
  double dt = (new_time_ - old_time_ ).toSec(); // [s]
  double dx = ( latest_posestamped_msg_.pose.position.x - last_used_posestamped_msg_.pose.position.x ) / dt;// [m/s]
  double dy = ( latest_posestamped_msg_.pose.position.y - last_used_posestamped_msg_.pose.position.y ) / dt;                           // [m/s]
  double dth = ( tf::getYaw(latest_posestamped_msg_.pose.orientation) - tf::getYaw(last_used_posestamped_msg_.pose.orientation) ) / dt;  // [度/s]
  geometry_msgs::Quaternion or_q = latest_posestamped_msg_.pose.orientation;

  nav_msgs::Odometry Odometry_msg_;
  geometry_msgs::TransformStamped Transform_msg_;

  Odometry_msg_.header.stamp = latest_posestamped_msg_.header.stamp;
  Odometry_msg_.header.frame_id = parent_id_;
  Odometry_msg_.child_frame_id = frame_id_;

  Odometry_msg_.pose.pose.position.x  = latest_posestamped_msg_.pose.position.x;
  Odometry_msg_.pose.pose.position.y  = latest_posestamped_msg_.pose.position.y;
  Odometry_msg_.pose.pose.position.z  = 0.0;
  Odometry_msg_.pose.pose.orientation = or_q;

  Odometry_msg_.pose.covariance[0]  =
  Odometry_msg_.pose.covariance[7]  =
  Odometry_msg_.pose.covariance[14] =
  Odometry_msg_.pose.covariance[21] =
  Odometry_msg_.pose.covariance[28] =
  Odometry_msg_.pose.covariance[35] = 0.031;

  Odometry_msg_.twist.twist.linear.x  =  dx;
  Odometry_msg_.twist.twist.linear.y  =  dy;
  Odometry_msg_.twist.twist.linear.z  =  0.0;
  Odometry_msg_.twist.twist.angular.x =  0.0;
  Odometry_msg_.twist.twist.angular.y =  0.0;
  Odometry_msg_.twist.twist.angular.z =  dth;

  Odometry_msg_.twist.covariance[0]  =
  Odometry_msg_.twist.covariance[7]  =
  Odometry_msg_.twist.covariance[14] =
  Odometry_msg_.twist.covariance[21] =
  Odometry_msg_.twist.covariance[28] =
  Odometry_msg_.twist.covariance[35] = 999;

  Transform_msg_.header.stamp    = latest_posestamped_msg_.header.stamp;
  Transform_msg_.header.frame_id = parent_id_;
  Transform_msg_.child_frame_id  = frame_id_;

  Transform_msg_.transform.translation.x = latest_posestamped_msg_.pose.position.x;
  Transform_msg_.transform.translation.y = latest_posestamped_msg_.pose.position.y;
  Transform_msg_.transform.translation.z = 0.0;
  Transform_msg_.transform.rotation      = or_q;

  last_used_posestamped_msg_ = latest_posestamped_msg_;
  old_time_ = new_time_;

  tf::TransformBroadcaster odom_broadcaster;
  odom_broadcaster.sendTransform(Transform_msg_);
  odom_pub.publish(Odometry_msg_);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pose2d2odom");
  ros::NodeHandle nh;
  ros::Subscriber posestamped_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/pose_stamped", 5,posestampedCallback );
  odom_pub = nh.advertise<nav_msgs::Odometry> ("/odom", 5);//;
  ros::spin ();
}
