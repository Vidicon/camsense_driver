#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <camsense_driver/camsense_x1.h>


CamsenseX1::CamsenseX1(const std::string& port, uint32_t baud_rate, float offset):
port_(port), baud_rate_(baud_rate), offset_(offset), serial_(io_, port_)
{
  shutting_down_ = false;
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

}

CamsenseX1::~CamsenseX1()
{

}

void CamsenseX1::parse()
{
  State state = SYNC0;

  bool got_scan = false;
  boost::array<uint8_t, 36> raw_bytes;
  
  const float IndexMultiplier = 400 / 360.0;
  float averageRotationSpeed = 0.0;

  float previousStart = 0.0;

  for(int i = 0; i < 400; i++)
  {
    distanceArray[i] = 0;
    qualityArray[i] = 0;
  }

  while (!shutting_down_ && !got_scan)
  {
  
    switch (state)
    {
    case SYNC0:
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[0],1));
      if (raw_bytes[0] == KSYNC0)
      {
        state = SYNC1;
      }
      break;

    case SYNC1:
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[1],1));
      if (raw_bytes[1] == KSYNC1)
      {
        state = SYNC2;
        break;
      }
      state = SYNC0;
      break;
    
    case SYNC2:
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2],1));
      if (raw_bytes[2] == KSYNC2)
      {
        state = SYNC3;
        break;
      }
      state = SYNC0;
      break;
    
    case SYNC3:
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[3],1));
      if (raw_bytes[3] == KSYNC3)
      {
        state = PARSE;
        break;
      }
      state = SYNC0;
      break;

    case PARSE:
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[4],32));

      float RotationSpeed = ((uint16_t) (raw_bytes[5] << 8) | raw_bytes[4]) / 3840.0; // 3840.0 = (64 * 60)
      if(averageRotationSpeed == 0.0)
      {
        averageRotationSpeed = RotationSpeed;
      }
      averageRotationSpeed = RotationSpeed * 0.01 + averageRotationSpeed * 0.99;
      
      float packageStartAngle =  ((uint16_t) (raw_bytes[7] << 8) | raw_bytes[6]) / 64.0 - 640;
      float packageEndAngle =  ((uint16_t) (raw_bytes[33] << 8) | raw_bytes[32]) / 64.0 - 640;

      float step = 0.0;
      if(packageEndAngle > packageStartAngle)
      {
          step = (packageEndAngle - packageStartAngle); 
      }
      else
      {
          step = (packageEndAngle - (packageStartAngle - 360)); 
      }
      step /= 8;

      for(int i = 0; i < 8; i++)
      {

        uint8_t distanceL = raw_bytes[8+(i*3)];
        uint8_t distanceH = raw_bytes[9+(i*3)];
        uint8_t quality = raw_bytes[10+(i*3)];

        float measurementAngle = (packageStartAngle + step * i) + (offset_ + 180);
        float fIndex = measurementAngle * IndexMultiplier;
        int index = round(fIndex);

        index = index % 400; // limit index between 0 and 399
        
        index = 399-index; // invert to match ROS

        distanceArray[index] = ((uint16_t) distanceH << 8) | distanceL;
        qualityArray[index] = quality;

        if(quality == 0)
        {
          distanceArray[index] = 0;
        }
      }

      if(previousStart > packageStartAngle)
      {
        got_scan = true;
        rotationSpeed_ = averageRotationSpeed;
        break;
      }
      previousStart = packageStartAngle;
      
      state = SYNC0;
      break;
    
    }
    
    
  }
  
}

float CamsenseX1::getRotationSpeed()
{
  return rotationSpeed_;
}
