#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <camsense_driver/camsense_x1.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "camsense_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  double offset;
  std::string frame_id;

  std_msgs::Float32 rpmsMsg;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("frame_id", frame_id, std::string("laser"));
  priv_nh.param("baud_rate", baud_rate, 115200); // Not really needed as camsense only works at 115200.
  priv_nh.param("offset", offset, 0.0);

  
  
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Publisher rpms_pub = n.advertise<std_msgs::Float32>("rpms",1000);

  ROS_INFO("Starting Camsense_publisher (%s@%i) with %f offset",port.c_str(),baud_rate,offset);

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = frame_id;
  scan.range_min = 0.0;
  scan.angle_max = 2.0*M_PI;
  scan.range_min = 0.12;
  scan.range_max = 8.0;
  scan.angle_increment = (2*M_PI)/400;
  scan.ranges.resize(400);
  scan.intensities.resize(400);
  
  CamsenseX1 camsense(port, baud_rate, offset);

  while (ros::ok())
  {

    camsense.parse();

    for(int i = 0; i < 400; i++)
    {
      scan.ranges[i] =  camsense.distanceArray[i] / 1000.0;
      scan.intensities[i] =  camsense.qualityArray[i];
    }
    scan.time_increment = 1.0 / camsense.getRotationSpeed() / 400;
    scan.header.stamp = ros::Time::now();
    scan_pub.publish(scan);

    rpmsMsg.data = camsense.getRotationSpeed();
    rpms_pub.publish(rpmsMsg);
  }
  camsense.close();

  return 0;
}
