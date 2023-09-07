#include "ros/ros.h"

#include "geographic_msgs/GeoPoseStamped.h"
#include "mdt_msgs/Gps.h"
#include "project11/utils.h"

ros::Publisher asv_pub;

ros::Subscriber geopose_sub;

void geoPoseCallback(const geographic_msgs::GeoPoseStamped::ConstPtr &msg)
{
  mdt_msgs::Gps asv_msg;
  asv_msg.header = msg->header;
  asv_msg.latitude = msg->pose.position.latitude;
  asv_msg.longitude = msg->pose.position.longitude;
  asv_msg.altitude = msg->pose.position.altitude;
  asv_msg.heading = project11::quaternionToHeadingDegrees(msg->pose.orientation);

  asv_pub.publish(asv_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geopose_to_asv");
  ros::NodeHandle n;

  asv_pub = n.advertise<mdt_msgs::Gps>("/auv/gps", 1);

  ros::Subscriber geopose_sub = n.subscribe("input", 1, geoPoseCallback);

  ros::spin();
  
  return 0;
}
