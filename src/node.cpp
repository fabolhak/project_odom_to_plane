#include "vector"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "project_odom_to_plane/tf2_geometry_msgs.h"

static std::string moving_frame, static_frame;

ros::Publisher odo_pub;
ros::Subscriber odo_sub;

tf::TransformBroadcaster* tf_broad;

static bool use_static_cov = false;
static std::vector<double> static_cov;


void odoCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // get roll and pitch of odometry in
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  /*
   * calculate transfrom to correct odometry
   * - rotate around negativ roll and pitch angle (so that they become zero)
   * - move along z axis (so that the z value becomes zero)
   */

  // transfrom negative roll and pitch angle to body coordinate system
  tf::Vector3 rpy_transformed = tf::Matrix3x3(q) * tf::Vector3(-roll, -pitch, 0);
  tf::Quaternion q1;
  q1.setRPY(rpy_transformed.x(), rpy_transformed.y(), rpy_transformed.z());

  // transform position to corrected coordinate system
  tf::Vector3 t_transformed =  tf::Matrix3x3(q1) * tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  t_transformed = tf::Vector3(0,0, -t_transformed.z());

  // create transform
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = t_transformed.x();
  t.transform.translation.y = t_transformed.y();
  t.transform.translation.z = t_transformed.z();
  tf::quaternionTFToMsg(q1, t.transform.rotation);

  // do the transform
  nav_msgs::Odometry new_msg;
  tf2::doTransform(msg->pose, new_msg.pose, t);

  // set correct frames
  new_msg.header.frame_id = static_frame;
  new_msg.child_frame_id = moving_frame;

  // set time
  new_msg.header.stamp = msg->header.stamp;

  // check wheater to use static covariances
  if(use_static_cov)
  {
    std::copy(static_cov.begin(), static_cov.end(), new_msg.pose.covariance.data());
  }

  // publish
  odo_pub.publish(new_msg);

  // tf
  tf::StampedTransform tf;
  tf::Transform tf_data;

  tf.child_frame_id_ = moving_frame;
  tf.frame_id_ = static_frame;
  tf.stamp_ = msg->header.stamp;
  tf_data.setOrigin( tf::Vector3(new_msg.pose.pose.position.x,
                                 new_msg.pose.pose.position.y,
                                 new_msg.pose.pose.position.z) );
  tf::Quaternion q3;
  tf::quaternionMsgToTF(new_msg.pose.pose.orientation, q3);
  tf_data.setRotation(q3);
  tf.setData(tf_data);
  tf_broad->sendTransform(tf);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "project_odom_to_plane");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get frames
  moving_frame = pnh.param<std::string>("moving_frame", "");
  static_frame = pnh.param<std::string>("static_frame", "");

  // setup subscriber and publisher
  tf_broad = new tf::TransformBroadcaster;
  odo_pub = pnh.advertise<nav_msgs::Odometry>("odom_out", 10);
  odo_sub = pnh.subscribe("odom_in", 10, odoCallback);

  // get static covariances
  use_static_cov = pnh.param<bool>("use_static_cov", false);
  if(use_static_cov)
  {
    if(!pnh.getParam("cov/static_pose_cov", static_cov))
    {
     ROS_ERROR_STREAM("Failed to load static covariances from file! Aborting");
     return 1;
    }
    ROS_INFO_STREAM("Use static covariances.");
  }


  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  delete tf_broad;

  return 0;
}
