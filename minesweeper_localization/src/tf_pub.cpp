#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

void callback(const geometry_msgs::Pose2D::ConstPtr& msg){

  x= msg->x;
  y= msg->y;
  th=msg->theta;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher_node");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe<geometry_msgs::Pose2D>("/minesweeper/pose", 100, callback);
  //ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 100, callback);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "minesweeper/odom";
    odom_trans.child_frame_id = "link_chassis";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    r.sleep();
  }
}

