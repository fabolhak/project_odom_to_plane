#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

static std::string moving_frame, static_frame;

ros::Publisher odo_pub;
ros::Subscriber odo_sub;

tf::TransformBroadcaster* tf_broad;


void odoCallback(const nav_msgs::OdometryConstPtr &msg)
{
  nav_msgs::Odometry new_msg = *msg;

  new_msg.header.frame_id = static_frame;
  new_msg.child_frame_id = moving_frame;

  // manually set z to zero
  new_msg.pose.pose.position.z = 0;
  new_msg.twist.twist.linear.z = 0;

  // manually set roll and pitch to zero
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(new_msg.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  q.setRPY(0, 0, yaw);
  tf::quaternionTFToMsg(q, new_msg.pose.pose.orientation);
  new_msg.twist.twist.angular.x = 0;
  new_msg.twist.twist.angular.y = 0;

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
  tf_data.setRotation(q);
  tf.setData(tf_data);
  tf_broad->sendTransform(tf);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "convert_zed_odom");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get frames
  moving_frame = pnh.param<std::string>("moving_frame", "");
  static_frame = pnh.param<std::string>("static_frame", "");

  // setup subscriber and publisher
  tf_broad = new tf::TransformBroadcaster;
  odo_pub = pnh.advertise<nav_msgs::Odometry>("odom_out", 10);
  odo_sub = pnh.subscribe("odom_in", 10, odoCallback);

  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  delete tf_broad;

  return 0;
}
