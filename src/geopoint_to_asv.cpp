#include "ros/ros.h"

#include "geographic_msgs/GeoPointStamped.h"
#include "mdt_msgs/Gps.h"

ros::Publisher asv_pub;

ros::Subscriber geopoint_sub;

void geoPointCallback(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
{
  mdt_msgs::Gps asv_msg;
  asv_msg.header = msg->header;
  asv_msg.latitude = msg->position.latitude;
  asv_msg.longitude = msg->position.longitude;
  asv_msg.altitude = msg->position.altitude;

  asv_pub.publish(asv_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geopoint_to_asv");
  ros::NodeHandle n;

  asv_pub = n.advertise<mdt_msgs::Gps>("/auv_gps", 1);

  ros::Subscriber geopoint_sub = n.subscribe("input", 1, geoPointCallback);

  ros::spin();
  
  return 0;
}
