#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "project11_msgs/PlatformList.h"

#include "drix_msgs/DrixsNetworkInfo.h"
#include "drix_msgs/LauncherDescriptionArray.h"
#include "mdt_msgs/LightGps.h"
#include "mdt_msgs/Gps.h"
#include "mdt_msgs/ShipSize.h"

#include <tf2/utils.h>

project11_msgs::Platform mothership_platform;

class Platform
{
public:
  void update(const drix_msgs::DrixNetworkInfo& data, ros::Time now)
  {
    m_platform.name = data.drix_name;
    m_platform.platform_namespace = "/project11/" + data.drix_name;
    //if(!m_gps_sub)
    {
      //m_gps_sub = n.subscribe("/"+data.drix_name+"/light_gps", 1, &Platform::gpsCallback, this);
      project11_msgs::NavSource nav;
      nav.name = "robobox";
      nav.position_topic = "/project11/robobox/"+data.drix_name+"/position";
      nav.orientation_topic = "/project11/robobox/"+data.drix_name+"/orientation";
      nav.velocity_topic = "/project11/robobox/"+data.drix_name+"/velocity";
      m_platform.nav_sources.push_back(nav);
      m_platform.length = 7.7;
      m_platform.width = 0.85;
      m_platform.reference_x = (m_platform.length/2.0)-3.2; // bow is 3.2m from phins ref
      m_platform.color.r = 1.0;
      m_platform.color.g = 0.0;
      m_platform.color.b = 0.0;
      m_platform.color.a = 1.0;
    }
    m_update_time = now;
  }

  void update(const cortix_msgs::LauncherDescription& data, ros::Time now)
  {
    m_platform.name = data.name;
    m_platform.platform_namespace = "/project11/" + data.name;
    //if(!m_gps_sub)
    {
      //m_gps_sub = n.subscribe("/launchers/"+data.name+"/gps", 1, &Platform::gpsCallback, this);
      project11_msgs::NavSource nav;
      nav.name = "robobox";
      nav.position_topic = "/project11/robobox/"+data.name+"/position";
      nav.orientation_topic = "/project11/robobox/"+data.name+"/orientation";
      m_platform.nav_sources.push_back(nav);
      m_platform.length = 8.3;
      m_platform.width = 2.51;
      m_platform.color.r = 0.5;
      m_platform.color.g = 0.5;
      m_platform.color.b = 0.5;
      m_platform.color.a = 1.0;
    }
    m_update_time = now;
  }

  void addToList(project11_msgs::PlatformList& list, ros::Time now, ros::Duration maxAge = ros::Duration(10))
  {
    if(now-m_update_time < maxAge)
      list.platforms.push_back(m_platform);
  }

  void gpsCallback(const mdt_msgs::LightGps::ConstPtr& data, ros::Time now, rosbag::Bag& outbag)
  {
    //if(m_position_pub)
    {
      sensor_msgs::NavSatFix nsf;
      nsf.header.stamp = data->stamp;
      nsf.latitude = data->latitude;
      nsf.longitude = data->longitude;
      //m_position_pub.publish(nsf);
      outbag.write(m_platform.nav_sources.front().position_topic, now, nsf);
    }
    double yaw = M_PI*(90-data->heading)/180.0;
    //if(m_orientation_pub)
    {
      sensor_msgs::Imu imu;
      imu.header.stamp = data->stamp;
      tf2::Quaternion q;
      q.setRPY(0,0,yaw);
      tf2::convert(q, imu.orientation);
      //m_orientation_pub.publish(imu);
      outbag.write(m_platform.nav_sources.front().orientation_topic, now, imu);
    }
    //if(m_velocity_pub)
    {
      double sin_yaw = sin(yaw);
      double cos_yaw = cos(yaw);
      geometry_msgs::TwistWithCovarianceStamped twcs;
      twcs.header.stamp = data->stamp;
      twcs.twist.twist.linear.x = data->sog*cos_yaw;
      twcs.twist.twist.linear.y = data->sog*sin_yaw;
      //m_velocity_pub.publish(twcs);
      outbag.write(m_platform.nav_sources.front().velocity_topic, now, twcs);
    }
  }

private:
  project11_msgs::Platform m_platform;
  ros::Time m_update_time;
};

std::map<std::string, Platform> platforms;

void drixNetworkInfoCallback(const drix_msgs::DrixsNetworkInfo::ConstPtr& data, ros::Time now)
{
  for(auto drix: data->drixs)
    platforms[drix.drix_name].update(drix, now);
}

void launcherDescriptionCallback(const drix_msgs::LauncherDescriptionArray::ConstPtr& data, ros::Time now)
{
  for(auto launcher: data->descriptions)
    platforms[launcher.name].update(launcher, now);
}

void publishPlatforms(ros::Time now, rosbag::Bag& outbag)
{
  project11_msgs::PlatformList pl;
  for(auto platform: platforms)
    platform.second.addToList(pl, now);
  if(!mothership_platform.name.empty())
    pl.platforms.push_back(mothership_platform);
  //platforms_pub.publish(pl);
  outbag.write("project11/platforms", now, pl);
}

void mothershipGpsCallback(const mdt_msgs::Gps::ConstPtr& data, ros::Time now, rosbag::Bag& outbag)
{
  if(mothership_platform.name.empty())
  {
    project11_msgs::NavSource nav;
    nav.name = "robobox";
    nav.position_topic = "/project11/robobox/mothership/position";
    nav.orientation_topic = "/project11/robobox/mothership/orientation";
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
  //mothership_position_pub.publish(nsf);
  outbag.write(mothership_platform.nav_sources.front().position_topic, now, nsf);

  sensor_msgs::Imu imu;
  imu.header = data->header;
  tf2::Quaternion q;
  double yaw = M_PI*(90-data->heading)/180.0;
  q.setRPY(0,0,yaw);
  tf2::convert(q, imu.orientation);
  //mothership_orientation_pub.publish(imu);
  outbag.write(mothership_platform.nav_sources.front().orientation_topic, now, imu);
}

void mothershipSizeCallback(const mdt_msgs::ShipSize::ConstPtr& data, ros::Time now, rosbag::Bag& outbag)
{
  mothership_platform.length = data->length;
  mothership_platform.width = data->width;
}

int main(int argc, char **argv)
{
  if(argc != 3)
  {
    std::cerr << "usage: robobox_bag_to_project11 in.bag out.bag" << std::endl;
    return 1;
  }

  rosbag::Bag outbag;
  outbag.open(argv[2], rosbag::bagmode::Write);
  outbag.setCompression(rosbag::CompressionType::LZ4 );

  rosbag::Bag inbag;
  inbag.open(argv[1], rosbag::bagmode::Read);

  ros::Time last_time;

  rosbag::View view(inbag);

  auto size = view.size();
  uint32_t count = 0;
  uint32_t increment = size/100;
  uint32_t last_report = 0;

  for(const auto m: rosbag::View(inbag))
  {
    if (count >= last_report+increment)
    {
      std::cerr << count << " of " << size << std::endl;
      last_report = count;
    }

    auto current_time = m.getTime();

    if(m.getTopic() == "/bridge_comm_masters/network_info")
    {
      drix_msgs::DrixsNetworkInfo::ConstPtr dni = m.instantiate<drix_msgs::DrixsNetworkInfo>();
      if(dni)
        drixNetworkInfoCallback(dni, current_time);
    }
    
    if(m.getTopic() == "/launchers/list")
    {
      drix_msgs::LauncherDescriptionArray::ConstPtr lda = m.instantiate<drix_msgs::LauncherDescriptionArray>();
      if(lda)
        launcherDescriptionCallback(lda, current_time);
    }

    if(m.getTopic() == "/mothership_gps")
    {
      mdt_msgs::Gps::ConstPtr gps = m.instantiate<mdt_msgs::Gps>();
      if(gps)
        mothershipGpsCallback(gps, current_time, outbag);
    }

    if(m.getTopic() == "/mothership_size")
    {
      mdt_msgs::ShipSize::ConstPtr ss = m.instantiate<mdt_msgs::ShipSize>();
      if(ss)
        mothershipSizeCallback(ss, current_time, outbag);
    }

    for(auto p: platforms)
    {
      if(m.getTopic() == "/"+p.first+"/light_gps" || m.getTopic() == "/launchers/"+p.first+"/gps")
      {
        mdt_msgs::LightGps::ConstPtr gps = m.instantiate<mdt_msgs::LightGps>();
        if(gps)
          p.second.gpsCallback(gps, current_time, outbag);
      }
    }

    if(current_time - last_time > ros::Duration(1.0))
    {
      if(last_time.isValid())
        publishPlatforms(current_time, outbag);
      last_time = current_time;
    }

    count += 1;
  }


  //platforms_pub = n.advertise<project11_msgs::PlatformList>("project11/platforms", 1);

  //ros::Subscriber drix_network_info_sub = n.subscribe("/bridge_comm_masters/network_info", 1, drixNetworkInfoCallback);
  //ros::Subscriber launcher_description_sub = n.subscribe("/launchers/list", 1, launcherDescriptionCallback);
  //ros::Subscriber mothership_gps_sub = n.subscribe("/mothership_gps", 1, mothershipGpsCallback);
  //ros::Subscriber mothership_size_sub = n.subscribe("/mothership_size", 1, mothershipSizeCallback);

  //ros::Timer timer = n.createTimer(ros::Duration(1.0), publishPlatforms);
  
  return 0;
}
  