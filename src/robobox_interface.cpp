#include "ros/ros.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "project11_msgs/PlatformList.h"

#include "cortix_msgs/BridgeCommFleetStatus.h"
#include "drix_msgs/LauncherDescriptionArray.h"
#include "mdt_msgs/LightGps.h"
#include "mdt_msgs/Gps.h"
#include "mdt_msgs/ShipSize.h"

#include <tf2/utils.h>

ros::Publisher platforms_pub;

ros::Subscriber mothership_gps_sub;
ros::Publisher mothership_position_pub;
ros::Publisher mothership_orientation_pub;
ros::Publisher mothership_velocity_pub;

project11_msgs::Platform mothership_platform;

class Platform
{
public:
  void update(const cortix_msgs::BridgeCommVehicleStatus& data)
  {
    m_platform.name = data.vehicle_name;
    m_platform.platform_namespace = "project11/" + data.vehicle_name;
    if(!m_gps_sub)
    {
      ros::NodeHandle n;
      m_gps_sub = n.subscribe("/"+data.vehicle_name+"/comms/topic_simplifier/light_gps", 1, &Platform::gpsCallback, this);
      project11_msgs::NavSource nav;
      nav.name = "robobox";
      nav.position_topic = "/project11/robobox/"+data.vehicle_name+"/position";
      m_position_pub = n.advertise<sensor_msgs::NavSatFix>(nav.position_topic, 1);
      nav.orientation_topic = "/project11/robobox/"+data.vehicle_name+"/orientation";
      m_orientation_pub = n.advertise<sensor_msgs::Imu>(nav.orientation_topic, 1);
      nav.velocity_topic = "/project11/robobox/"+data.vehicle_name+"/velocity";
      m_velocity_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>(nav.velocity_topic, 1);
      m_platform.nav_sources.push_back(nav);
      m_platform.length = 7.7;
      m_platform.width = 0.85;
      m_platform.reference_x = (m_platform.length/2.0)-3.2; // bow is 3.2m from phins ref
      m_platform.color.r = 1.0;
      m_platform.color.g = 0.0;
      m_platform.color.b = 0.0;
      m_platform.color.a = 1.0;
    }
    m_update_time = ros::Time::now();
  }

  void update(const cortix_msgs::LauncherDescription& data)
  {
    m_platform.name = data.name;
    m_platform.platform_namespace = "project11/" + data.name;
    if(!m_gps_sub)
    {
      ros::NodeHandle n;
      m_gps_sub = n.subscribe("/launcher/"+data.name+"/gps", 1, &Platform::gpsCallback, this);
      project11_msgs::NavSource nav;
      nav.name = "robobox";
      nav.position_topic = "/project11/robobox/"+data.name+"/position";
      m_position_pub = n.advertise<sensor_msgs::NavSatFix>(nav.position_topic, 1);
      nav.orientation_topic = "/project11/robobox/"+data.name+"/orientation";
      m_orientation_pub = n.advertise<sensor_msgs::Imu>(nav.orientation_topic, 1);
      m_platform.nav_sources.push_back(nav);
      m_platform.length = 8.3;
      m_platform.width = 2.51;
      m_platform.color.r = 0.5;
      m_platform.color.g = 0.5;
      m_platform.color.b = 0.5;
      m_platform.color.a = 1.0;
    }
    m_update_time = ros::Time::now();
  }

  void addToList(project11_msgs::PlatformList& list, ros::Duration maxAge = ros::Duration(10))
  {
    if(ros::Time::now()-m_update_time < maxAge)
      list.platforms.push_back(m_platform);
  }

  void gpsCallback(const mdt_msgs::LightGps::ConstPtr& data)
  {
    if(m_position_pub)
    {
      sensor_msgs::NavSatFix nsf;
      nsf.header.stamp = data->stamp;
      nsf.latitude = data->latitude;
      nsf.longitude = data->longitude;
      m_position_pub.publish(nsf);
    }
    double yaw = M_PI*(90-data->heading)/180.0;
    if(m_orientation_pub)
    {
      sensor_msgs::Imu imu;
      imu.header.stamp = data->stamp;
      tf2::Quaternion q;
      q.setRPY(0,0,yaw);
      tf2::convert(q, imu.orientation);
      m_orientation_pub.publish(imu);
    }
    if(m_velocity_pub)
    {
      double sin_yaw = sin(yaw);
      double cos_yaw = cos(yaw);
      geometry_msgs::TwistWithCovarianceStamped twcs;
      twcs.header.stamp = data->stamp;
      twcs.twist.twist.linear.x = data->sog*cos_yaw;
      twcs.twist.twist.linear.y = data->sog*sin_yaw;
      m_velocity_pub.publish(twcs);
    }
  }

private:
  project11_msgs::Platform m_platform;
  ros::Time m_update_time;
  ros::Subscriber m_gps_sub;
  ros::Publisher m_position_pub;
  ros::Publisher m_orientation_pub;
  ros::Publisher m_velocity_pub;
};

std::map<std::string, Platform> platforms;

void drixNetworkInfoCallback(const cortix_msgs::BridgeCommFleetStatus::ConstPtr& data)
{
  for(auto drix: data->vehicles)
    platforms[drix.vehicle_name].update(drix);
}

void launcherDescriptionCallback(const drix_msgs::LauncherDescriptionArray::ConstPtr& data)
{
  for(auto launcher: data->descriptions)
    platforms[launcher.name].update(launcher);
}

void publishPlatforms(const ros::TimerEvent& event)
{
  project11_msgs::PlatformList pl;
  for(auto platform: platforms)
    platform.second.addToList(pl);
  if(!mothership_platform.name.empty())
    pl.platforms.push_back(mothership_platform);
  platforms_pub.publish(pl);
}

void mothershipGpsCallback(const mdt_msgs::Gps::ConstPtr& data)
{
  if(mothership_platform.name.empty())
  {
    ros::NodeHandle n;
    project11_msgs::NavSource nav;
    nav.name = "robobox";
    nav.position_topic = "/project11/robobox/mothership/position";
    mothership_position_pub = n.advertise<sensor_msgs::NavSatFix>(nav.position_topic, 1);
    nav.orientation_topic = "/project11/robobox/mothership/orientation";
    mothership_orientation_pub = n.advertise<sensor_msgs::Imu>(nav.orientation_topic, 1);
    mothership_platform.nav_sources.push_back(nav);
    mothership_platform.name = "mothership";
    mothership_platform.color.r = 0.0;
    mothership_platform.color.g = 0.0;
    mothership_platform.color.b = 1.0;
    mothership_platform.color.a = 1.0;
  }

  sensor_msgs::NavSatFix nsf;
  nsf.header = data->header;
  nsf.latitude = data->latitude;
  nsf.longitude = data->longitude;
  nsf.altitude = data->altitude;
  mothership_position_pub.publish(nsf);

  sensor_msgs::Imu imu;
  imu.header = data->header;
  tf2::Quaternion q;
  double yaw = M_PI*(90-data->heading)/180.0;
  q.setRPY(0,0,yaw);
  tf2::convert(q, imu.orientation);
  mothership_orientation_pub.publish(imu);

}

void mothershipSizeCallback(const mdt_msgs::ShipSize::ConstPtr& data)
{
  mothership_platform.length = data->length;
  mothership_platform.width = data->width;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robobox_interface");
  ros::NodeHandle n;

  platforms_pub = n.advertise<project11_msgs::PlatformList>("platforms", 1);

  ros::Subscriber drix_network_info_sub = n.subscribe("/comms/bridge_comm_masters/network_info", 1, drixNetworkInfoCallback);
  ros::Subscriber launcher_description_sub = n.subscribe("/launcher/launchers_manager/list", 1, launcherDescriptionCallback);
  ros::Subscriber mothership_gps_sub = n.subscribe("/mothership/gps", 1, mothershipGpsCallback);
  ros::Subscriber mothership_size_sub = n.subscribe("/lever_arms/mothership_cfg/mothership_size", 1, mothershipSizeCallback);

  ros::Timer timer = n.createTimer(ros::Duration(1.0), publishPlatforms);
  ros::spin();
  
  return 0;
}
  